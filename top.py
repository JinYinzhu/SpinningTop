from dm_control import mujoco
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

string1 = """
<mujoco model="tippe top">
  <option integrator="RK4"/>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" 
     rgb2=".2 .3 .4" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
  </asset>

  <worldbody>
    <geom size=".2 .2 .01" type="plane" material="grid"/>
    <light pos="0 0 1"/>
    <camera name="closeup" pos="0 -.1 .07" xyaxes="1 0 0 0 1 2"/>
    <body name="top" pos="0 0 .02">
      <freejoint/>
      <geom name="ball" type="sphere" size=".02" />
      <geom name="stem" type="cylinder" pos="0 0 .02" size="0.004 .008"/>
      <geom name="ballast" type="box" size=".023 .023 0.005"  pos="0 0 -.015" 
       contype="0" conaffinity="0" group="3"/>
    </body>
  </worldbody>
  
  <keyframe>
"""
string2 = """
  </keyframe>
</mujoco>
"""

def random_quaternion():
    #generates a quaternion of a random angle
    target = np.random.normal(size=3)
    target /= np.linalg.norm(target)
    initial = np.array([0,0,1])
    axis = np.cross(target,initial)
    axis /= np.linalg.norm(axis)
    cos_theta = np.dot(target,initial)
    theta = np.arccos(cos_theta)
    q1 = np.cos(theta/2)
    sin_half_theta = np.sin(theta/2)
    q2 = axis[0]*sin_half_theta
    q3 = axis[1]*sin_half_theta
    q4 = axis[2]*sin_half_theta
    return str(q1)+' '+str(q2)+' '+str(q3)+' '+str(q4)

def generate_random_top(n, mute=True):
    #generates n random orientation tops
    imgs = []
    for i in range(n):
        if not mute and i%10==0:
            print(i/10)
        qtn = random_quaternion()
        tippe_top = string1+"""    <key name="spinning" qpos="0 0 0.03 """+qtn+"""" qvel="0 0 0 0 0 0" />"""+string2
        physics = mujoco.Physics.from_xml_string(tippe_top)
        physics.reset(0)
        pixels = physics.render(camera_id='closeup')
        imgs.append(pixels)
    return imgs
    
def get_quaternion(target):
    #given an orientation in form of 3-d vector, returns its quaternion form
    target /= np.linalg.norm(target)
    initial = np.array([0,0,1])
    axis = np.cross(target,initial)
    axis /= np.linalg.norm(axis)
    cos_theta = np.dot(target,initial)
    theta = np.arccos(cos_theta)
    q1 = np.cos(theta/2)
    sin_half_theta = np.sin(theta/2)
    q2 = axis[0]*sin_half_theta
    q3 = axis[1]*sin_half_theta
    q4 = axis[2]*sin_half_theta
    return str(q1)+' '+str(q2)+' '+str(q3)+' '+str(q4)

def generate_spec_top(position,orientation):
    #generate a top of specified position and orientation
    #position and orientation: array of 3 numbers (x,y,z)
    qtn = get_quaternion(orientation)
    pos_string = str(position[0])+' '+str(position[1])+' '+str(position[2])
    tippe_top = string1+"""    <key name="spinning" qpos=" """+pos_string+" "+qtn+"""" qvel="0 0 0 0 0 0" />"""+string2
    #print(tippe_top)
    physics = mujoco.Physics.from_xml_string(tippe_top)
    physics.reset(0)
    pixels = physics.render(camera_id='closeup')
    return pixels


#example
top = generate_spec_top(np.array([0.,0.,0.03]),np.array([0.,1.,0.]))
plt.imshow(top)
plt.show()

#another example
tops = generate_random_top(5)
fig,ax = plt.subplots(1, 5, figsize=(15, 5))
for i in range(5):
    ax[i].axis('off')
    ax[i].imshow(tops[i])
plt.show()