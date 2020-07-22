#!/usr/bin/env python
# run with /usr/bin/python src/semantic_segmentation/src/semantic_segmentation_node.py
import rospy
import rospkg
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import tensorflow as tf
import tarfile
import os
import csv

INPUT_SIZE = 257


class DeepLabModel(object):
    """Class to load deeplab model and run inference."""

    INPUT_TENSOR_NAME = 'ImageTensor:0'
    OUTPUT_TENSOR_NAME = 'SemanticPredictions:0'
    FROZEN_GRAPH_NAME = 'frozen_inference_graph'

    def __init__(self, tarball_path):
        """Creates and loads pretrained deeplab model."""
        self.graph = tf.Graph()

        graph_def = None
        # Extract frozen graph from tar archive.
        tar_file = tarfile.open(tarball_path)
        for tar_info in tar_file.getmembers():
            if self.FROZEN_GRAPH_NAME in os.path.basename(tar_info.name):
                file_handle = tar_file.extractfile(tar_info)
                graph_def = tf.compat.v1.GraphDef.FromString(file_handle.read())
                break

        tar_file.close()

        if graph_def is None:
            raise RuntimeError('Cannot find inference graph in tar archive.')

        with self.graph.as_default():
            tf.import_graph_def(graph_def, name='')

        self.sess = tf.compat.v1.Session(graph=self.graph)

    def run(self, image):
        """Runs inference on a single image.

        Args:
          image: An OpenCV image

        Returns:
          resized_image: RGB image resized from original input image.
          seg_map: Segmentation map of `resized_image`.
        """
        center = (image.shape[0] / 2, image.shape[1] / 2)
        cropped = image[
                  center[0] - INPUT_SIZE / 2: center[0] + INPUT_SIZE / 2,
                  center[1] - INPUT_SIZE / 2: center[1] + INPUT_SIZE / 2,
                  :
                  ]
        batch_seg_map = self.sess.run(
            self.OUTPUT_TENSOR_NAME,
            feed_dict={self.INPUT_TENSOR_NAME: [cropped]})
        seg_map = batch_seg_map[0]
        return cropped, seg_map


rospack = rospkg.RosPack()
model_dir = os.path.join(rospack.get_path('semantic_segmentation'), 'models')
deeplab_path = os.path.join(model_dir,
                            'deeplabv3_mnv2_ade20k_train_2018_12_03.tar.gz')
seg_cat_path = os.path.join(model_dir,
                            'objectInfo150.csv')
model = DeepLabModel(deeplab_path)
bridge = CvBridge()
segmented_img_pub = rospy.Publisher("/camera/color/segmented",
                                    sensor_msgs.msg.Image, queue_size=1)
viz_img_pub = rospy.Publisher("/camera/color/segmented/viz",
                              sensor_msgs.msg.Image, queue_size=1)

seg_cat = ['background']
seg_color = [[0, 0, 0]]
with open(seg_cat_path, 'r') as csvfile:
    cat_csv_reader = csv.reader(csvfile)
    next(cat_csv_reader)
    for row in cat_csv_reader:
        seg_cat.append(row[-1])
print('Loaded ' + str(len(seg_cat)) + ' categories')

def image_callback(img_msg):
    try:
        img = bridge.imgmsg_to_cv2(img_msg=img_msg)
    except CvBridgeError as e:
        rospy.logerr(str(e))
        return

    resized_im, seg_map = model.run(img)

    segmented_img = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
    center = (img.shape[0] / 2, img.shape[1] / 2)
    segmented_img[center[0] - INPUT_SIZE / 2: center[0] + INPUT_SIZE / 2,
    center[1] - INPUT_SIZE / 2: center[1] + INPUT_SIZE / 2] = np.array(seg_map).astype(np.uint8)
    #segmented_img = np.array(seg_map).astype(np.uint8)
    segmented_img_msg = bridge.cv2_to_imgmsg(segmented_img)
    segmented_img_pub.publish(segmented_img_msg)

def main():
    rospy.init_node('semantic_segmentation')
    image_sub = rospy.Subscriber('/camera/color/image_raw',
                                 sensor_msgs.msg.Image, image_callback,
                                 queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
