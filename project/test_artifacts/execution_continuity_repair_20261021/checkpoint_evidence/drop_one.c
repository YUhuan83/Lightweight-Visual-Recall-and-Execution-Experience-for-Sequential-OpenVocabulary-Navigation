#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>
static pthread_once_t init_once=PTHREAD_ONCE_INIT;
static ssize_t (*orig_from)(int,void*,size_t,int,struct sockaddr*,socklen_t*);
static ssize_t (*orig_msg)(int,struct msghdr*,int);
static uint64_t until=0;
static pthread_mutex_t fault_lock=PTHREAD_MUTEX_INITIALIZER;
static void init(void){orig_from=dlsym(RTLD_NEXT,"recvfrom");orig_msg=dlsym(RTLD_NEXT,"recvmsg");}
static uint64_t mono(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000000ull+t.tv_nsec;}
static uint64_t number(const char*b,size_t n,const char*k){const char*p=memmem(b,n,k,strlen(k));if(!p)return 0;p+=strlen(k);while((size_t)(p-b)<n&&(*p==' '||*p==':'))p++;return strtoull(p,0,10);}
static int discard(const char*b,size_t n,int flags){
 if(flags&MSG_PEEK||n<24||memcmp(b,"RTPS",4)||!memmem(b,n,"wildos_source_control_v1",24))return 0;
 if(number(b,n,"\"sequence\"")!=25)return 0;
 uint64_t t=mono();pthread_mutex_lock(&fault_lock);if(!until)until=t+650000000ull;int drop=t<until;pthread_mutex_unlock(&fault_lock);
 if(drop){const char*p=getenv("WILDOS_DROP_LOG");if(p){FILE*f=fopen(p,"a");if(f){fprintf(f,"{\"pid\":%d,\"sequence\":25,\"mono_ns\":%llu}\n",getpid(),(unsigned long long)t);fclose(f);}}}return drop;
}
ssize_t recvfrom(int fd,void*b,size_t n,int flags,struct sockaddr*a,socklen_t*l){pthread_once(&init_once,init);ssize_t r;do{r=orig_from(fd,b,n,flags,a,l);}while(r>0&&(size_t)r<=n&&discard(b,r,flags));return r;}
ssize_t recvmsg(int fd,struct msghdr*m,int flags){pthread_once(&init_once,init);size_t cn=m->msg_controllen;socklen_t an=m->msg_namelen;ssize_t r;for(;;){m->msg_controllen=cn;m->msg_namelen=an;r=orig_msg(fd,m,flags);if(!(r>0&&m->msg_iovlen==1&&(size_t)r<=m->msg_iov[0].iov_len&&discard(m->msg_iov[0].iov_base,r,flags)))return r;}}
