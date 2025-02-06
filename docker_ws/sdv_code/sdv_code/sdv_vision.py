import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import torch
import torchvision.transforms as T
import cv2
import numpy as np
from PIL import Image as PILImage

# Create a color map for the segmentation classes
def create_pascal_label_colormap():
    colormap = np.zeros((256, 3), dtype=int)
    ind = np.arange(256, dtype=int)

    for shift in reversed(range(8)):
        for channel in range(3):
            colormap[:, channel] |= ((ind >> channel) & 1) << shift
        ind >>= 3

    return colormap

# Apply the colormap to the segmentation mask
def label_to_color_image(label):
    colormap = create_pascal_label_colormap()
    return colormap[label]

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/rgb_front/image',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/segmented_image', 10)
        self.bridge = CvBridge()
        self.model = torch.hub.load('pytorch/vision:v0.10.0', 'deeplabv3_resnet101', weights='DeepLabV3_ResNet101_Weights.COCO_WITH_VOC_LABELS_V1')
        self.model.eval()
        self.preprocess = T.Compose([
            T.Resize(513),
            T.CenterCrop(513),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def listener_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            self.get_logger().info(f"Image encoding: {msg.encoding}")

            #Aqui el codigo de la practica5

        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Convert image to PIL format
        #Aqui el codigo de la practica5
        
        # Preprocess image
        #Aqui el codigo de la practica5
     
        #Apply torch model to image
        #Aqui el codigo de la practica5 
        # with torch.no_grad():
        #     output = self.model("your preprocessed image")['out'][0]

        # Get class IDs per pixel
        #Aqui el codigo de la practica5 

        # Get unique class IDs present in the image
        #Aqui el codigo de la practica5 

        # Convert class IDs to readable labels
        #Aqui el codigo de la practica5 

        # Convert the segmentation mask to a color image
        #Aqui el codigo de la practica5 

        # Convert back to ROS Image message and publish
        #Aqui el codigo de la practica5

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
