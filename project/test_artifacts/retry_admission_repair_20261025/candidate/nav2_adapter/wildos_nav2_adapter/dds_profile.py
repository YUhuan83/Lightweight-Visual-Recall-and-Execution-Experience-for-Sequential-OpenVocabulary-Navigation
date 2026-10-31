"""Explicit loopback FastDDS transport for the co-located BeamNG test stack.

This module opens no ROS context. Apply before starting sensors, navigation,
vision or a recorder. ROS_LOCALHOST_ONLY=1 would append another UDP/SHM pair
in Humble rmw_fastrtps 6.2.10, overriding the intended transport isolation.
"""
import argparse
import hashlib
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def profile_errors(data):
    """Require loopback UDP with no implicit transports or QoS overrides."""
    try:
        if len(data)>65536:return ['DDS profile exceeds bounded size']
        root=ET.fromstring(data)
        for element in root.iter():element.tag=element.tag.split('}')[-1]
        descriptors=root.findall('./transport_descriptors/transport_descriptor')
        participants=root.findall('./participant')
        if root.tag!='profiles' or len(descriptors)!=1 or len(participants)!=1:
            return ['DDS profile requires one explicit transport/participant']
        descriptor=descriptors[0];participant=participants[0]
        transport_id=descriptor.findtext('transport_id')
        errors=[]
        if descriptor.findtext('type')!='UDPv4':errors.append('DDS transport must be UDPv4')
        if [a.text for a in descriptor.findall('./interfaceWhiteList/address')]!=['127.0.0.1']:
            errors.append('DDS transport must be restricted to loopback')
        if participant.get('is_default_profile')!='true':errors.append('DDS participant must be default')
        if participant.findtext('./rtps/useBuiltinTransports')!='false':errors.append('DDS builtin transports must be disabled')
        if not transport_id or [t.text for t in participant.findall('./rtps/userTransports/transport_id')]!=[transport_id]:
            errors.append('DDS participant transport reference differs')
        if any(e.tag in ('data_writer','data_reader','publisher','subscriber','qos') for e in root.iter()):
            errors.append('DDS profile must not override endpoint QoS')
        return errors
    except (ET.ParseError,TypeError,ValueError):return ['Invalid DDS XML profile']


def local_dds_environment(profile, inherited):
    """Return a fresh child environment; never change the caller's mapping."""
    path=Path(profile).resolve(strict=True);errors=profile_errors(path.read_bytes())
    if errors:raise ValueError('; '.join(errors))
    env=dict(inherited)
    if env.get('RMW_IMPLEMENTATION','') not in ('','rmw_fastrtps_cpp'):
        raise ValueError('Local FastDDS profile cannot override a different RMW implementation')
    env.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp',ROS_LOCALHOST_ONLY='0',
               FASTRTPS_DEFAULT_PROFILES_FILE=str(path))
    return env


def environment_errors(environment, expected_profile, *, profile_reader=None):
    """Check actual process environment against the frozen test profile."""
    errors=[]
    if environment.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':errors.append('RMW implementation differs')
    if environment.get('ROS_LOCALHOST_ONLY')!='0':errors.append('ROS localhost mode would append SHM')
    path=environment.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    if not path:return errors+['DDS profile path missing']
    try:
        data=profile_reader(path) if profile_reader else Path(path).read_bytes()
        errors.extend(profile_errors(data))
        if hashlib.sha256(data).digest()!=hashlib.sha256(expected_profile).digest():
            errors.append('DDS profile bytes differ from frozen source')
    except (OSError,ValueError,TypeError):errors.append('DDS profile cannot be read')
    return errors


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path)
    parser.add_argument('command',nargs=argparse.REMAINDER)
    args=parser.parse_args();command=args.command
    if command and command[0]=='--':command=command[1:]
    if not command:parser.error('Supply a command to run in the local DDS environment')
    profile=args.profile
    if profile is None:
        from ament_index_python.packages import get_package_share_directory
        profile=Path(get_package_share_directory('wildos_nav2_adapter'))/'config/fastdds_local_udp.xml'
    env=local_dds_environment(profile,os.environ)
    os.execvpe(command[0],command,env)


if __name__=='__main__':main()
