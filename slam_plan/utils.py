
import numpy as np

from sensor_msgs.msg import PointField
from cv_bridge import CvBridge, CvBridgeError


DUMMY_FIELD_PREFIX = '__'

type_mappings = [
  (PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')),
  (PointField.INT16, np.dtype('int16')), (PointField.UINT16, np.dtype('uint16')),
  (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
  (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))
]
pftype_to_nptype = dict(type_mappings)
pftype_sizes = {
  PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2,
  PointField.UINT16: 2, PointField.INT32: 4, PointField.UINT32: 4,
  PointField.FLOAT32: 4, PointField.FLOAT64: 8
}

def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def dtype_to_fields(dtype):
    '''Convert a numpy record datatype into a list of PointFields.
    '''
    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = np.prod(shape)
            np_field_type = item_dtype
        else:
            pf.count = 1

    pf.datatype = nptype_to_pftype[np_field_type]
    pf.offset = field_offset
    fields.append(pf)
    return fields

def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray

    Reshapes the returned array to have shape (height, width), even if the height is 1.

    The reason for using np.frombuffer rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
      [fname for fname, _type in dtype_list
        if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]
    ]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
     a 3xN matrix.
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

    # pull out x, y, and z values
    points = np.zeros(cloud_array.shape + (3,), dtype=dtype)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']

    return points
