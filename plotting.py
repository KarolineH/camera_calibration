import matplotlib.pyplot as plt
from pytransform3d.transform_manager import TransformManager

def plot_transforms(transforms):
    tm = TransformManager()
    if len(transforms.shape) == 3:
        print('Transforms shape:', transforms.shape)
        num_poses = transforms.shape[0]
        for i in range(num_poses):
            pose = transforms[i, :, :]
            tm.add_transform(f'Pose {i + 1}','world', pose)
    elif len(transforms.shape) == 2:
        print('Transforms shape:', transforms.shape)
        tm.add_transform(f'Pose {i + 1}','world', transforms)
        
    ax = tm.plot_frames_in("world", s=0.1)
    ax.set_xlim((-0.25, 0.75))
    ax.set_ylim((-0.5, 0.5))
    ax.set_zlim((0.0, 1.0))
    plt.show()