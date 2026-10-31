import cv2
import re
from typing import Dict, List
import numpy as np
import torch
import torch.nn.functional as F

from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from object_search_msgs.msg import ObjectMaskWithTf
from .semantic_mask import supported_mask_components


_COLOR_WORDS = (
    'black', 'blue', 'brown', 'gray', 'green', 'orange', 'pink', 'purple',
    'red', 'white', 'yellow',
)


def color_counterfactual_queries(query):
    """Build same-object prompts that differ only in a stated color attribute."""
    normalized = ' '.join(str(query).casefold().split())
    present = [word for word in _COLOR_WORDS
               if re.search(r'\b' + re.escape(word) + r'\b', normalized)]
    if len(present) != 1:
        return []
    selected = present[0]
    pattern = re.compile(r'\b' + re.escape(selected) + r'\b')
    return [pattern.sub(word, normalized, count=1)
            for word in _COLOR_WORDS if word != selected]


def localize_query(
    text_feats: torch.Tensor,
    spatial_feats: torch.Tensor,
    orig_img_shape: tuple,
    pixel_level_seg: bool,
    mask_threshold: float,
    negative_text_feats: torch.Tensor = None,
    minimum_competitive_cells: int = 0,
    semantic_margin: float = 0.0,
    minimum_component_cells: int = 0,
):
    """
    Localizes the text query in the spatial features.
    
    :param text_feats: (num_queries, D) Text feature vectors.
    :param spatial_feats: (B, D, H, W) Spatial feature map from the image.
    :param orig_img_shape: (H, W) Original image shape.
    :param pixel_level_seg: Whether to perform pixel-level segmentation.
    :param mask_threshold: Threshold for converting similarity maps to binary masks.

    :return: similarity_map, binary_mask
    """
    # Normalize features
    spatial_feats = spatial_feats / spatial_feats.norm(dim=1, keepdim=True)
    text_feats = text_feats.to(dtype=spatial_feats.dtype)

    # Compare target and likely visual confounders before expanding the mask
    # to image resolution. At 30x45 RADIO features, a 1-cell false match can
    # otherwise become a large patch and create a plausible 3-D landmark.
    text_sim_spatial = torch.einsum('qc,bchw->bqhw', text_feats, spatial_feats)
    binary_features = text_sim_spatial > mask_threshold
    if negative_text_feats is not None and minimum_competitive_cells > 0:
        negative_text_feats = negative_text_feats.to(dtype=spatial_feats.dtype)
        negative_sim = torch.einsum('qc,bchw->bqhw', negative_text_feats, spatial_feats)
        binary_features &= text_sim_spatial - negative_sim.max(dim=1, keepdim=True).values > semantic_margin
        counts = binary_features.flatten(1).sum(dim=1)
        binary_features &= (counts >= minimum_competitive_cells).view(-1, 1, 1, 1)

    if minimum_component_cells > 0:
        supported = supported_mask_components(
            binary_features.detach().cpu().numpy(), minimum_component_cells)
        binary_features = torch.as_tensor(supported, device=binary_features.device)
    
    # Resize similarity maps to match the original image size
    if pixel_level_seg:
        interp_mode = 'bilinear'
    else:
        interp_mode = 'nearest'
    text_sim_spatial = F.interpolate(
        text_sim_spatial, size=(orig_img_shape[0], orig_img_shape[1]), mode=interp_mode
    ).cpu().numpy()  # Shape: (B, num_queries, H, W)

    # Resize the already verified target mask with the same sampling as the
    # similarity visualization, preserving a zero mask when evidence is weak.
    binary_mask = F.interpolate(
        binary_features.float(), size=(orig_img_shape[0], orig_img_shape[1]),
        mode=interp_mode
    ).cpu().numpy().astype(np.uint8)

    return text_sim_spatial, binary_mask

def convert_maskmsg_to_multiarray(mask_msg: np.ndarray) -> UInt8MultiArray:
    """
    Converts a binary mask to a UInt8MultiArray message.

    :param mask_msg: (B, 1, H, W) Binary mask of the detected object.
    :return: UInt8MultiArray message.
    """
    multiarray_msg = UInt8MultiArray()
    multiarray_msg.data = mask_msg.flatten().tolist()
    multiarray_msg.layout.data_offset = 0
    b,c,h,w = mask_msg.shape
    multiarray_msg.layout.dim = [
        MultiArrayDimension(label='batch', size=b, stride=b * c * h * w),
        MultiArrayDimension(label='channel', size=c, stride=c * h * w),
        MultiArrayDimension(label='height', size=h, stride=h * w),
        MultiArrayDimension(label='width', size=w, stride=w),
    ]
    return multiarray_msg

def get_objectmask_msg(
    binary_mask: np.ndarray,
    cam_inverted: bool,
    odom_msg: Odometry,
    tf_data: List[TransformStamped],
    cam_info_msgs: List[CameraInfo],
    query: str = "",
) -> ObjectMaskWithTf:
    """
    Converts the binary mask and associated data into an ObjectMaskWithTf message.

    :param binary_mask: (3, 1, H, W) Binary mask of the detected object.
    :param cam_inverted: Whether the camera is inverted.
    :param odom_msg: Odometry message for the robot's pose.
    :param tf_data: List of TF data for frame transformations from camera to odom.
    :param cam_info_msgs: List of CameraInfo messages for the cameras.

    :return: ObjectMaskWithTf message.
    """        
    
    if cam_inverted:
        binary_mask = np.rot90(binary_mask, k=2, axes=(2, 3))

    obj_mask_msg = ObjectMaskWithTf()
    obj_mask_msg.header = odom_msg.header
    obj_mask_msg.query = query

    obj_mask_msg.odom = odom_msg
    obj_mask_msg.cam_infos = cam_info_msgs
    obj_mask_msg.object_mask = convert_maskmsg_to_multiarray(binary_mask)
    obj_mask_msg.cam_transforms = TFMessage(transforms=tf_data)

    return obj_mask_msg
