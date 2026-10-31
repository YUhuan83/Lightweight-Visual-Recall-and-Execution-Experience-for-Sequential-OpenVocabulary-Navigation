#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

// Diagnostic only: identical libc receive arguments/results; never logs payload.
static pthread_once_t once=PTHREAD_ONCE_INIT;
static ssize_t (*real_recvfrom)(int,void*,size_t,int,struct sockaddr*,socklen_t*);
static ssize_t (*real_recvmsg)(int,struct msghdr*,int);
static int output=-1;
static void initialize(void) {
  real_recvfrom=dlsym(RTLD_NEXT,"recvfrom");real_recvmsg=dlsym(RTLD_NEXT,"recvmsg");
  const char *path=getenv("WILDOS_NATIVE_RX_TRACE_FILE");if(!path)return;
  char cmd[8192];int fd=open("/proc/self/cmdline",O_RDONLY);if(fd<0)return;
  ssize_t n=read(fd,cmd,sizeof(cmd));close(fd);
  if(n<=0 || !memmem(cmd,(size_t)n,"/native_vehicle_control",22))return;
  output=open(path,O_WRONLY|O_CREAT|O_EXCL|O_APPEND,0600);
}
static uint64_t stamp(clockid_t clock) {
  struct timespec ts;clock_gettime(clock,&ts);return (uint64_t)ts.tv_sec*1000000000ull+ts.tv_nsec;
}
static uint64_t number(const char *buf,size_t size,const char *key) {
  const char *p=memmem(buf,size,key,strlen(key));if(!p)return 0;
  p+=strlen(key);char tmp[32];size_t n=0;
  while((size_t)(p-buf)<size && (*p==' ' || *p==':'))p++;
  while((size_t)(p-buf)<size && *p>='0' && *p<='9' && n<sizeof(tmp)-1)tmp[n++]=*p++;
  tmp[n]=0;return strtoull(tmp,NULL,10);
}
static uint64_t ancillary_stamp(struct msghdr *msg) {
  for(struct cmsghdr *c=CMSG_FIRSTHDR(msg);c;c=CMSG_NXTHDR(msg,c)) {
    if(c->cmsg_level==SOL_SOCKET && c->cmsg_type==SO_TIMESTAMPNS && c->cmsg_len>=CMSG_LEN(sizeof(struct timespec))) {
      struct timespec ts;memcpy(&ts,CMSG_DATA(c),sizeof(ts));
      return (uint64_t)ts.tv_sec*1000000000ull+ts.tv_nsec;
    }
  }
  return 0;
}
static void trace(int fd,const char *buf,size_t size,uint64_t returned_wall,uint64_t returned_mono,uint64_t kernel_ns) {
  if(output<0 || size<20 || memcmp(buf,"RTPS",4))return;
  uint64_t issued=number(buf,size,"\"issued_wall_ns\"");if(!issued)return;
  uint64_t seq=number(buf,size,"\"sequence\"");
  char line[512];int n=snprintf(line,sizeof(line),"{\"pid\":%ld,\"tid\":%ld,\"fd\":%d,\"bytes\":%zu,\"issued_wall_ns\":%llu,\"sequence\":%llu,\"kernel_wall_ns\":%llu,\"recv_wall_ns\":%llu,\"recv_mono_ns\":%llu}\n",
    (long)getpid(),(long)syscall(SYS_gettid),fd,size,(unsigned long long)issued,(unsigned long long)seq,
    (unsigned long long)kernel_ns,(unsigned long long)returned_wall,(unsigned long long)returned_mono);
  if(n>0 && n<(int)sizeof(line))syscall(SYS_write,output,line,(size_t)n);
}
ssize_t recvfrom(int fd,void *buf,size_t size,int flags,struct sockaddr *addr,socklen_t *addrlen) {
  pthread_once(&once,initialize);
  int type=0;socklen_t type_len=sizeof(type);
  int datagram=output>=0 && getsockopt(fd,SOL_SOCKET,SO_TYPE,&type,&type_len)==0 && type==SOCK_DGRAM;
  uint64_t kernel_ns=0;ssize_t n;
  if(datagram && (!addr || addrlen)) {
    int enabled=1;setsockopt(fd,SOL_SOCKET,SO_TIMESTAMPNS,&enabled,sizeof(enabled));
    union {struct cmsghdr align;char buf[CMSG_SPACE(sizeof(struct timespec))];} control;
    struct iovec iov={.iov_base=buf,.iov_len=size};
    struct msghdr msg={.msg_name=addr,.msg_namelen=addr?*addrlen:0,.msg_iov=&iov,.msg_iovlen=1,
      .msg_control=control.buf,.msg_controllen=sizeof(control.buf)};
    n=real_recvmsg(fd,&msg,flags);
    if(n>=0) {kernel_ns=ancillary_stamp(&msg);if(addrlen && addr)*addrlen=msg.msg_namelen;}
  } else n=real_recvfrom(fd,buf,size,flags,addr,addrlen);
  int saved=errno;
  uint64_t wall=stamp(CLOCK_REALTIME),mono=stamp(CLOCK_MONOTONIC);
  if(n>0 && (size_t)n<=size)trace(fd,buf,(size_t)n,wall,mono,kernel_ns);
  errno=saved;return n;
}
ssize_t recvmsg(int fd,struct msghdr *msg,int flags) {
  pthread_once(&once,initialize);
  ssize_t n=real_recvmsg(fd,msg,flags);int saved=errno;
  uint64_t wall=stamp(CLOCK_REALTIME),mono=stamp(CLOCK_MONOTONIC);
  if(n>0 && msg->msg_iovlen==1 && (size_t)n<=msg->msg_iov[0].iov_len)
    trace(fd,msg->msg_iov[0].iov_base,(size_t)n,wall,mono,ancillary_stamp(msg));
  errno=saved;return n;
}
