import numpy as np
from tf import transformations


def transform_coordinates(position_child, quaternion_child):
    """
    将子坐标系(东北天)中的位置和姿态转换到基坐标系(北东地)

    参数:
    position_child: numpy.array, 形状为 (3,), 子坐标系中的位置 [x, y, z]
    quaternion_child: numpy.array, 形状为 (4,), 子坐标系中的四元数 [x, y, z, w]

    返回:
    position_base: numpy.array, 形状为 (3,), 基坐标系中的位置
    quaternion_base: numpy.array, 形状为 (4,), 基坐标系中的四元数
    """

    # 1. 位置转换
    position_base = np.array([position_child[1], position_child[0], -position_child[2]])

    # 2. 姿态转换 (四元数)
    # 从子坐标系到基坐标系的旋转四元数
    q_child_to_base = transformations.quaternion_from_euler(0, 0, -np.pi / 2, 'rxyz')

    # 计算基坐标系中的四元数
    quaternion_base = transformations.quaternion_multiply(q_child_to_base, quaternion_child)

    return position_base, quaternion_base


# 测试函数
def test_transform():
    # 创建测试数据
    position_child = np.array([1.0, 2.0, 3.0])  # 子坐标系中的位置
    quaternion_child = np.array([0.0, 0.0, 0.0, 1.0])  # 子坐标系中的四元数 (单位四元数)

    # 执行转换
    position_base, quaternion_base = transform_coordinates(position_child, quaternion_child)

    print("子坐标系中的位置:", position_child)
    print("基坐标系中的位置:", position_base)
    print("子坐标系中的四元数:", quaternion_child)
    print("基坐标系中的四元数:", quaternion_base)

    # 将四元数转换为欧拉角以便更直观地理解
    euler_child = transformations.euler_from_quaternion(quaternion_child)
    euler_base = transformations.euler_from_quaternion(quaternion_base)

    print("子坐标系中的欧拉角 (弧度):", euler_child)
    print("基坐标系中的欧拉角 (弧度):", euler_base)


# 运行测试
test_transform()