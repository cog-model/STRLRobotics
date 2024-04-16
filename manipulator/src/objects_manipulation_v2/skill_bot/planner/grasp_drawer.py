import numpy as np
import io
import trimesh
from PIL import Image

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from trimesh.transformations import translation_matrix, rotation_matrix
class PoseDrawer:
    def __init__(self, model_path) -> None:
        self.topic = rospy.Publisher('/manipulator/grasp_view', RosImage, queue_size=1)
        self.mesh_model : trimesh.Trimesh = trimesh.load_mesh(model_path) # type: ignore
        self.mesh_model.apply_scale(10**-3) # type: ignore
        self.mesh_model.apply_transform(rotation_matrix(-np.math.pi/2, [1,0,0])) # type: ignore
        self.mesh_model.apply_transform(translation_matrix([-0.05,-0.02,0])) # type: ignore
        self.scene = trimesh.Scene()
        self.scene.add_geometry(self.mesh_model, node_name='mesh_node',geom_name='mesh')
        self.scene.graph.update('cam', self.scene.graph.base_frame, matrix = [
        [1, 0, 0, 0],
        [ 0, -1, 0, 0],
        [ 0, 0, -1, 0],
        [ 0, 0,  0, 1]])
        self.cam = self.scene.set_camera()
        self.cam.name = 'cam'
    
    def build_msg(self, img):
        msg = RosImage()
        msg.header.stamp = rospy.Time.now()
        msg.height, msg.width, *_ = img.shape
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 3*msg.width
        msg.data = img.tobytes()
        return msg
    
    def show(self, img, pose, camera_info : CameraInfo):
        self.cam.K = np.reshape(camera_info.K, (3,3))
        self.cam.resolution = (camera_info.width, camera_info.height)

        self.scene.graph.update('mesh_node', self.scene.graph.base_frame , matrix = pose)
        
        render_img = self.scene.save_image(visible=False)
        render_img = np.array(Image.open(io.BytesIO(render_img)))
        render_img[np.all(render_img==255, -1), :] = 0
        img = np.copy(img)
        img[np.all(render_img!=0, -1) ] = 0
        img += render_img[:,:, :3]
        
        msg = self.build_msg(img)
        self.topic.publish(msg)
        return img
    