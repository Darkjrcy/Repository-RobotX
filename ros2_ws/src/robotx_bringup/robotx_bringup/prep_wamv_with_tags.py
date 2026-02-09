# Import ROS2 library:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import shutil
from pathlib import Path

# Have a template of the format need to be added in the WAM-V urdf:
APRILTAG_BLOCK_TEMPLATE = """
  <!--AprilTag link {idx}-->
  <link name="${{namespace}}/aptag{idx}">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual> 
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://wamv_description/models/{apriltag_name}{idx}/meshes/{apriltag_name}{idx}.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <mesh filename="package://wamv_description/models/{apriltag_name}{idx}/meshes/{apriltag_name}{idx}.dae"/>
      </geometry>
    </collision>
  </link>

  <!--Apply the joint {idx}-->
  <joint name="${{namespace}}/aptag_joint{idx}" type="fixed">
      <parent link="${{namespace}}/base_link"/>
      <child link="${{namespace}}/aptag{idx}"/>
      <origin rpy="0 1.5707963 3.1415926" xyz="{x} {y} {z}"/>
  </joint>
"""



# Function to change teh position strings to numbers that can be inuted for the AprilTag position:
def parse_positions(pos_str, num):
    """
    Accept:
      "0.28,0,1.76" for 1 tag
      "0.28,0,1.76; 0.30,0.1,1.76; ..." for multiple
    """
    # Divide the psoitions in chunks
    chunks = [c.strip() for c in pos_str.split(';') if c.strip()]

    # Save the psoitions in a lsit
    poses = []
    for c in chunks:
        # Get each position in the x, y, z coordiantes of the model base.
        parts = [p.strip() for p in c.split(',')]
        if len(parts) != 3:
            raise ValueError(f"Bad position chunk '{c}'. Use 'x,y,z' or separate with ';'")
        poses.append(tuple(float(p) for p in parts))

    # If only one pose provided but num > 1, repeat it
    if len(poses) == 1 and num > 1:
        poses = poses * num

    # Return the positions of the apriltags:
    return poses[:num]



# Function to add the apriltag sdf in a urdf file:
def patch_xacro(main_xacro, copy_path, num_april, pos_str, name_april):
    # Define the xacro file path:
    xacro_path = Path(main_xacro)
    copy_xacro = Path(copy_path)

    # Identify the text inside the xacro file:
    text = copy_xacro.read_text(encoding="utf-8")

    # Add the Apriltags:
    # Find the end of the robot tag to apply the apriltags:
    end_idx = text.rfind("</robot>")
    # Define the positions of the APriltags:
    poses = parse_positions(pos_str, num_april)
    # Write the apriltags in a string:
    blocks = []
    for i, (x, y, z) in enumerate(poses):
        blocks.append(APRILTAG_BLOCK_TEMPLATE.format(idx=i, x=x, y=y, z=z, apriltag_name=name_april))
    insert_payload = "\n".join(blocks) + "\n"
    # Add the payload in the main sdf:
    new_text = text[:end_idx] + insert_payload + text[end_idx:]

    # Atomic replace:
    tmp = xacro_path.with_suffix(xacro_path.suffix + ".tmp")
    tmp.write_text(new_text, encoding="utf-8")
    os.replace(str(tmp), str(xacro_path))  # atomic on same filesystem



# Start the One spin node:
class PrepApriltags(Node):
    # Start the Node:
    def __init__(self):
        super().__init__('prep_wamv_with_tags')

        # Declare the characteristics of the required AprilTags:
        self.declare_parameter('add_apriltags', True)
        self.declare_parameter('num_apriltags', 1)
        self.declare_parameter('position_respect_wamv_center', "0.28, 0, 1.76")
        self.declare_parameter('apriltag_name','marker')
        self.declare_parameter('urdf', '')
        self.declare_parameter('sim_mode', 'full')

        # Define the varibles with teh argumetns:
        self.add_apriltags = bool(self.get_parameter('add_apriltags').value)
        self.num_apriltags = int(self.get_parameter('num_apriltags').value)
        self.apriltags_positions = self.get_parameter('position_respect_wamv_center').value
        self.apriltag_name = self.get_parameter('apriltag_name').value
        self.urdf = self.get_parameter('urdf').value
        self.sim_mode = self.get_parameter('sim_mode').value
    


    # Function to write the apriltags in the urdf only once:
    def write_once(self):
        try:

            # Varible to see if hte urdf is changed:
            urdf_changed = False
            if not self.urdf:
                urdf_path = os.path.join(get_package_share_directory('wamv_description'),
                            'urdf', 'wamv_base.urdf.xacro')
                urdf_path_copy = os.path.join(get_package_share_directory('wamv_description'),
                                            'urdf', 'wamv_base_copy.urdf.xacro')

            # Create clean copy ONCE
            if not os.path.exists(urdf_path_copy):
                shutil.copy2(urdf_path, urdf_path_copy)

            if self.add_apriltags and (self.sim_mode in ('full', 'sim')) and (not self.urdf):
                patch_xacro(urdf_path, urdf_path_copy,
                            self.num_apriltags, self.apriltags_positions, self.apriltag_name)
                urdf_changed = True
                self.get_logger().info("AprilTags were added")
            else:
                # Restore original from clean copy
                shutil.copy2(urdf_path_copy, urdf_path)

        except Exception as e:
            self.get_logger().error(f"Failed to patch wamv xacro: {e}")



def main():
    rclpy.init()
    node = PrepApriltags()
    try:
        node.write_once()  
    finally:
        node.destroy_node()
        rclpy.shutdown()

        
        

