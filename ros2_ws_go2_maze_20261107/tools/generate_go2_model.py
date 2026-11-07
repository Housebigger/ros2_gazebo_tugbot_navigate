#!/usr/bin/env python3
"""Build the native Gazebo Go2 model from the pinned official URDF conversion."""
from copy import deepcopy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/tugbot_maze'))
from tugbot_maze.legged.params import STAND_POSE

root=Path(__file__).resolve().parents[1]
tree=ET.parse(sys.argv[1])
model=tree.getroot().find('model')
model.set('name','go2')
mapping={}
for source,target in (('fl','LF'),('fr','RF'),('rl','LH'),('rr','RH')):
    for old,new in (('hip','HAA'),('thigh','HFE'),('knee','KFE')):
        mapping[f'{source}_{old}']=f'{target}_{new}'
for joint in model.findall('joint'):
    joint.set('name',mapping[joint.get('name')])
for element in model.iter():
    for key in ('relative_to','attached_to','expressed_in'):
        if element.get(key) in mapping:
            element.set(key,mapping[element.get(key)])
for mesh in model.findall('.//mesh/uri'):
    mesh.text='model://go2/dae/'+Path(mesh.text).name
for collision in model.findall('link/collision'):
    if 'foot' in collision.get('name',''):
        surface=ET.SubElement(collision,'surface')
        ode=ET.SubElement(ET.SubElement(surface,'friction'),'ode')
        ET.SubElement(ode,'mu').text='1.0'
        ET.SubElement(ode,'mu2').text='1.0'
def plugin(filename,name,parameters):
    item=ET.SubElement(model,'plugin',filename=filename,name=name)
    for key,value in parameters.items():
        ET.SubElement(item,key).text=str(value)
    return item
for joint in model.findall('joint'):
    name=joint.get('name')
    effort=float(joint.find('axis/limit/effort').text)
    plugin('gz-sim-joint-position-controller-system','gz::sim::systems::JointPositionController',{
        'joint_name':name,'p_gain':320.0,'d_gain':4.0,'i_gain':0,
        'cmd_max':effort,'cmd_min':-effort,'initial_position':STAND_POSE[name],
        'topic':f'/model/go2/joint/{name}/cmd_pos',
    })
plugin('gz-sim-joint-state-publisher-system','gz::sim::systems::JointStatePublisher',{
    'topic':'/model/go2/joint_state',
})
plugin('gz-sim-odometry-publisher-system','gz::sim::systems::OdometryPublisher',{
    'odom_frame':'odom','robot_base_frame':'base_link','odom_publish_frequency':30,
    'dimensions':3,'odom_topic':'/odom','tf_topic':'/tf',
})
stock=ET.parse(root/'src/tugbot_description/models/tugbot/model.sdf').getroot()
base=model.find("link[@name='base_link']")
for name,pose in (('lidar_3d','0 0 0.25 0 0 0'),('camera_front','0.35 0 0.05 0 0 0')):
    sensor=deepcopy(stock.find(f".//sensor[@name='{name}']"))
    sensor.find('pose').text=pose
    frame=sensor.find('gz_frame_id')
    if frame is None:
        frame=ET.SubElement(sensor,'gz_frame_id')
    frame.text=f'go2/base_link/{name}'
    base.append(sensor)
world_path=root/'src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf'
world_tree=ET.parse(world_path)
include=world_tree.getroot().find('.//world/include')
include.find('uri').text='model://go2'
include.find('name').text='go2'
include.find('pose').text='-11.011281 -9.025070 0.46 0 0 0'
ET.indent(world_tree)
world_tree.write(root/'src/tugbot_gazebo/worlds/go2_maze.sdf',encoding='unicode',xml_declaration=True)
test=ET.parse(root/'src/tugbot_gazebo/worlds/anymal_test_world.sdf')
include=test.getroot().find('.//world/include')
include.find('uri').text='model://go2'
include.find('name').text='go2'
include.find('pose').text='0 0 0.46 0 0 0'
world=test.getroot().find('world')
world.set('name','go2_test')
physics=ET.SubElement(world,'physics',name='1ms',type='ignored')
ET.SubElement(physics,'max_step_size').text='0.001'
ET.SubElement(physics,'real_time_factor').text='1'
ET.indent(test)
test.write(root/'src/tugbot_gazebo/worlds/go2_test.sdf',encoding='unicode',xml_declaration=True)
ET.indent(tree)
tree.write(root/'src/tugbot_description/models/go2/model.sdf',encoding='unicode',xml_declaration=True)
