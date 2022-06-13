import cv2
import pybullet as p
import numpy as np
import math
import time
from pathlib import Path
import open3d


class Camera:
    """Camera
    This implementation is following OpenGL camera conventions and
    is preferred over the ROS/OpenCV camera implementation, as it
    is consistent with the rendering and Pybullet world.
    Check this for some OpenGL terminologies:
    http://www.songho.ca/opengl/gl_transform.html
    """

    def __init__(self, width, height):
        """
        Args:
            width (int)                 : Image width.
            height (int)                : Image height.
        """
        self._width = width
        self._height = height
        self._view_mat = None
        self._projection_mat = None
        self._near = None
        self._far = None
        self._fovy = None
        self._aspect = None
        self._camera_eye_pos = None
        self._camera_target_pos = None
        self._camera_up_vec = None

    def set_view_matrix_from_robot_link(self,robot,link):
        """
        Sets the camera view matrix (OpenGL ModelView Matrix).
        Args:
            robot(int)        : robot id
            link(int)         : link index
        """

        link_info = p.getLinkState(robot, link, 1, 1)
        link_pos = link_info[0]
        link_ori = link_info[1]
        rot = p.getMatrixFromQuaternion(link_ori)
        rot = np.array(rot).reshape(3,3)

        local_camera_x_vec = np.array([1, 0, 0])
        local_camera_z_vec = np.array([0, 0, 1])

        self._camera_eye_pos = link_pos + np.dot(rot, 0.01*
                local_camera_x_vec)
        self._camera_target_pos = link_pos + np.dot(rot, 10*
                local_camera_x_vec)
        self._camera_up_vec = np.dot(rot,local_camera_z_vec)

        # use the Pybullet helper function to compute the view matrix
        view_mat = p.computeViewMatrix(cameraEyePosition=self._camera_eye_pos,
                                       cameraTargetPosition=self._camera_target_pos,
                                       cameraUpVector=self._camera_up_vec)

        # Bullet is using column major matrices
        self._view_mat = np.reshape(view_mat, (4, 4)).transpose()

    def set_view_matrix(self, camera_eye_pos, camera_target_pos, camera_up_vec):
        """
        Sets the camera view matrix (OpenGL ModelView Matrix).
        Args:
            camera_eye_pos (list or np.array)       : Position of the center of the camera.
            camera_target_pos (list or np.array)    : Position of the center of the scene where the camera is looking at
            camera_up_vec (list or np.array)        : Camera up vector.
        """
        self._camera_eye_pos = camera_eye_pos
        self._camera_target_pos = camera_target_pos
        self._camera_up_vec = camera_up_vec

        # use the Pybullet helper function to compute the view matrix
        view_mat = p.computeViewMatrix(cameraEyePosition=camera_eye_pos,
                                       cameraTargetPosition=camera_target_pos,
                                       cameraUpVector=camera_up_vec)

        # Bullet is using column major matrices
        self._view_mat = np.reshape(view_mat, (4, 4)).transpose()

    def set_projection_matrix(self, fovy, aspect, near, far):
        """
        Sets the camera projection matrix (OpenGL Projection Matrix).
        Args:
            fovy (float)        : The vertical lens opening angle.
            aspect (float)      : The aspect ratio (width/height) of the lens.
            near (float)        : Distance to the front clipping plane.
            far (float)         : Distance to the back clipping plane.
        """
        self._fovy = fovy
        self._aspect = aspect
        self._near = near
        self._far = far

        # assuming bottom=-top and left=-right
        top = math.tan(math.radians(fovy / 2)) * near
        right = top * aspect

        self._projection_mat = np.array([
            [near / right,  0,              0,                              0],
            [0,             near / top,     0,                              0],
            [0,             0,              -(far + near) / (far - near),   -2 * far * near / (far - near)],
            [0,             0,              -1,                             0]
        ])


    def project_3D_to_pixel(self, point):
        """
        Projects a 3D point into the camera canvas.
        The pixel is in window coordinates, so depth is in the range [near, far].
        `normalize_depth` should be called to map [near, far] to [0, 1].
        This is the inverse of `project_pixel_to_3D`.
        Args:
            point (list or np.array)        : [x; y; z] 3D point in world coordinates.
        Returns:
            np.array                        : Pixel [x_w; y_w; z_w] in window coordinates.
        """
        # dirty way to transform point to homogeneous coordinates
        point = np.reshape(point, (3, -1))
        p = np.pad(point, ((0, 1), (0, 0)), mode='constant', constant_values=1)

        # transform point to eye coordinates
        point_eye = np.matmul(self.view_matrix, p)
        # transform point to clip coordinates
        point_clip = np.matmul(self.projection_matrix, point_eye)
        # transform point to normalized device coordinates
        ndc_point = self.clip_to_ndc(point_clip)
        # transform point to window coordinates
        pixel = self.ndc_to_window(ndc_point)

        return pixel

    def project_pointcloud_to_canvas(self, xyz, colors):
        """
        Projects a pointcloud to the window canvas. It further normalizes the depth
        to [0, 1] from [near, far] to be consistent with Pybullet camera.
        This is the inverse of `unproject_canvas_to_pointcloud`.
        Args:
            xyz (np.array)              : Point cloud in the format (3, N_points) with columns being [x; y; z].
            colors (np.array)           : Color of the points (3, N_points) with columns being [R; G; B].
        Returns:
            (np.array, np.array)        : Normalized Depth image and RGB image
        """
        pixels = self.project_3D_to_pixel(xyz)

        # create a canvas for storing the image
        canvas_depth = np.ones((self._height, self._width))
        canvas_rgb = np.ones((self.height, self.width, 4))

        # TODO: Can you do this in a faster and more Pythonic way? Possible solution: scipy.misc.toimage)
        for i in range(pixels.shape[1]):
            x_w = int(round(pixels[0, i]))
            y_w = int(round(pixels[1, i]))
            z_w = pixels[2, i]
            if 0 <= x_w < self._width and 0 <= y_w < self._height:
                canvas_depth[y_w, x_w] = z_w
                canvas_rgb[y_w, x_w, :] = colors[:, i].reshape(-1)

        # normalize depth
        return canvas_rgb.astype(np.uint8), self.normalize_depth(canvas_depth)

    def unproject_pixel_to_3Dray(self, pixel):
        """
        Returns the unit vector of the raycast passing from the camera center
        through the rectified pixel [u; v].
        This is useful if one wants to render image with raytacing.
        Args:
            pixel (list or np.array)    : [x_w; y_w] of the pixel in the window coordinates.
        Returns:
            np.array                    : [x; y; z] of the unit vector of the ray.
        """
        # TODO
        pass

    def unproject_pixel_to_3D(self, pixel):
        """
        Projects a pixel in window coordinates to a 3D point in world coordinates.
        Note that depth of the pixel (z_w) should be in window coordinates (in [near, far]).
        If you have normalized depth buffer (z_b), you should first call `denormalize_depth` method.
        This is the inverse of `project_3D_to_pixel`.
        Args:
            pixel (list or np.array)    : [x_w; y_w; z_w] of the pixel in window coordinates.
        Returns:
            np.array                    : [x; y; z] 3D point in world coordinates.
        """
        # transform point to normalized device coordinates
        pixel_ndc = self.window_to_ndc(pixel)

        # dirty way to transform point to homogeneous coordinates
        point = np.reshape(pixel_ndc, (3, -1))
        p = np.pad(point, ((0, 1), (0, 0)), mode='constant', constant_values=1.)

        m = np.matmul(self.projection_matrix, self.view_matrix)
        m_inv = np.linalg.inv(m)

        point = np.matmul(m_inv, p)
        return self.clip_to_world(point)

    def unproject_canvas_to_pointcloud(self, rgb_img, depth_img):
        """
        Generates a point cloud from the RGB image and depth buffer.
        Note that the depth buffer (coming from Pybullet which is
        in turn coming from OpenGL) stores the depth in normalized
        buffer form (z_b) and is in the range of [0, 1]. So we need to
        denormalize it.
        This is the invserse of `project_pointcloud_to_canvas`.
        Args:
            rgb_img (np.array)          : Input RGB image.
            depth_img (np.array)        : Input Depth buffer.
        Returns:
            (np.array, np.array)        : Point cloud in the (3, N_points) format with columns being [x; y; z;] and
                                        colors of the points in the (3, N_points) format with columns being [R; G; B].
        """
        # the depth buffer should be first transformed from normalized (z_b) to window (z_w)
        depth_img = self.denormalize_depth(depth_img)

        # create a canvas for storing x_w, y_w, z_w
        x = np.linspace(0, self.width - 1, self.width)
        y = np.linspace(0, self.height - 1, self.height)
        x_mesh, y_mesh = np.meshgrid(x, y)

        canvas = np.zeros((3, self._width * self._height))
        canvas[0, :] = x_mesh.reshape(-1)
        canvas[1, :] = y_mesh.reshape(-1)
        canvas[2, :] = depth_img.reshape(-1)

        colors = rgb_img.transpose(2, 0, 1).reshape(4, -1)
        pointcloud = self.unproject_pixel_to_3D(canvas)

        filtered_pointcloud = self.filter_pointcloud(pointcloud, self._far, negative=True)

        return filtered_pointcloud, colors

    def filter_pointcloud(self, input_cloud, value, negative):
        """
        Pass through filter that eliminates all points of a pointcloud below a specific threshold,
        along a specific axis. Notice that this operation is done in the current pointcloud frame
        Args:
            input_cloud (np.array, np.array)            : input pointcloud
            value (float)                               : Threshold value
            axis (np.array)                             : Axis
            negative (bool)                             : True will save the points < value, opposite behavior with False
        Returns:
            pointcloud_filtered (np.array, np.array)    : Filtered pointcloud
        """
        # normalize axis vector
        # axis = axis / np.linalg.norm(axis)

        remove = list()
        for i in range(np.shape(input_cloud)[1]):
            point = input_cloud[:, i]
            # d = np.dot(point, axis)
            d = np.linalg.norm(point)
            if negative and d > value:
                remove.append(i)
            elif not negative and d < value:
                remove.append(i)

        pointcloud_filtered = np.delete(input_cloud, remove, axis=1)
        return pointcloud_filtered



    def clip_to_ndc(self, point):
        """
        Transforms a point in clipped coordinates to normalized device coordinates.
        [x_c; y_c; z_c; w_c] --> [x_c/w_c; y_c/w_c; z_c/w_c]
        All values are now in the range [-1, 1].
        Values outside this range mean that they will not appear in the camera.
        Args:
            point (np.array)        : Point [x_c; y_c; z_c; w_c] in clipped coordinates.
        Returns:
            np.array                : Point [x_ndc; y_ndc; z_ndc] in normalized device coordinates.
        """
        return point[:3] / point[-1]

    def clip_to_world(self, point):
        """
        Transforms a point in clipped coordinates to the world coordinates.
        [x_c; y_c; z_c; w_c] --> [x_c/w_c; y_c/w_c; z_c/w_c]
        This is doing the same thing as `clip_to_ndc`, but has a different meaning and
        can be thought of as its inverse.
        It is also here for API consistency.
        Args:
            point (np.array)        : Point [x_c; y_c; z_c; w_c] in clipped coordinates.
        Returns:
            np.array                : Point [x; y; z] in world coordinates.
        """
        return point[:3] / point[-1]

    def ndc_to_window(self, point):
        """
        Applies viewpoint transformation to fit normalized device coordinates into rendering window.
        x: [-1, 1] --> [0, w]
        y: [-1, 1] --> [0, h]
        z: [-1, 1] --> [n, f]
        This is the inverse of `window_to_ndc`.
        Args:
            point (np.array)        : Point [x_ndc; y_ndc; z_ndc] in normalized device coordinates.
        Returns:
            np.array                : Pixel [x_w; y_w; z_w] in window coordinates.
        """
        scale = np.array([
            [self._width / 2],
            [self._height / 2],
            [(self._far - self._near) / 2]
        ])
        offset = np.array([
            [self._width / 2],
            [self._height / 2],
            [(self._far + self._near) / 2]
        ])
        return scale * point + offset

    def window_to_ndc(self, pixel):
        """
        Transforms a pixel from window coordinates to normalized device coordinates.
        This is the inverse of `ndc_to_window`.
        Args:
            pixel (np.array)        : Pixel [x_w; y_w; z_w] in window coordinates.
        Returns:
            np.array                : Point [x_ndc; y_ndc; z_ndc] in normalized device coordinates.
        """
        scale = np.array([
            [2. / self._width],
            [2. / self._height],
            [2. / (self._far - self._near)]
        ])
        offset = np.array([
            [-1.],
            [-1.],
            [-(self._far + self._near) / (self._far - self._near)]
        ])
        return scale * pixel + offset

    def depth_buffer_to_real(self, z_b):
        """
        Transforms the normalized depth buffer (z_b) to real depth (z_e).
        Unlike `unproject_pixel_to_3D`, this does not care about the pixel (u, v).
        Thus, it is faster to run in cases where the depth is only important.
        This is the inverse of `real_depth_to_buffer`.
        Args:
            z_b (float or np.array)     : Normalized depth z_b as stored in the OpenGL buffer.
        Returns:
            float or np.array           : Real depth z_e.
        """
        # from http://web.archive.org/web/20130416194336/http://olivers.posterous.com/linear-depth-in-glsl-for-real
        # basically undo the projection matrix multiplication of the Z component
        z_e = 2 * self._far * self._near / (self._far + self._near - (self._far - self._near) * (2 * z_b - 1))
        return z_e

    def real_depth_to_buffer(self, z_e):
        """
        Transforms the real depth (z_e) to normalized depth buffer (z_b).
        Unlike `project_3D_to_pixel`, this does not care about the pixel (u, v).
        Thus, it is faster to run in cases where the depth is only important.
        This is the inverse of `depth_buffer_to_real`.
        Args:
            z_e (float or np.array)     : Read depth z_e.
        Returns:
            float or np.array           : Normalized depth z_b as stored in the OpenGL buffer.
        """
        A = self._projection_mat[2, 2]
        B = self._projection_mat[2, 3]
        return 0.5 * (-A + B / z_e) + 0.5

    def normalize_depth(self, z_w):
        """
        Transforms a depth from window coordinates [near, far]
        to the normalized depth buffer [0, 1].
        Pybullet (OpenGL) camera is outputting the normalized depth buffer.
        This is the inverse of `denormalize_depth`.
        Args:
            z_w (float or np.array)     : Depth z_w in window coordinates.
        Returns:
            float or np.array           : Normalized depth buffer z_b.
        """
        return (z_w - self._near) / (self._far - self._near)

    def denormalize_depth(self, z_b):
        """
        Transforms a depth from normalized depth buffer [0, 1]
        to window coordinates [near, far].
        Pybullet (OpenGL) camera is outputting the normalized depth buffer,
        so you have to denormalize it first if you want to get the
        true depth using `project_pixel_to_3D`.
        This is the inverse of `normalize_depth`.
        Args:
            z_b (float or np.array)         : Normalized depth buffer z_b.
        Returns:
            float or np.array               : Depth z_w in window coordinates.
        """
        return (self._far - self._near) * z_b + self._near

    def get_pybullet_image(self):
        """
        Gets the pybullet simulated camera image.
        Returns:
            (np.array, np.array, np.array)      : RGB image, depth image, and segmentation mask
        """
        # Bullet is using column major matrices so be careful when reshaping
        _, _, rgb_img, depth_img, seg_img = p.getCameraImage(self._width,
                                                             self._height,
                                                             self._view_mat.reshape(-1, order='F'),
                                                             self._projection_mat.reshape(-1, order='F'),
                                                             renderer=p.ER_BULLET_HARDWARE_OPENGL)
        return rgb_img, depth_img, seg_img

    def get_raytraced_image(self):
        """
        Renders the image with raytracing.
        Returns:
            (np.array, np.array, np.array, np.array)    : RGB image, depth image, surface normal and segmentation mask
        """
        # TODO
        pass

    def show_image(self, img, RGB=True, save=False, title='Image'):
        """
        Shows the input image using OpenCV.
        Args:
            img (np.array)          : Input image.
            RGB (bool)              : If true, the input image is RGB, otherwise it is assumed to be a depth image.
            save (bool)             : If true, saves the image.
            title (str)             : Title of the window.
        """
        img = np.reshape(img, (self._height, self._width, -1))
        time_str = time.strftime("%Y%m%d-%H%M%S")
        if RGB:
            cv2.imshow(title, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            if save:
                cv2.imwrite(title + '_' + time_str + '.png', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        else:
            cv2.imshow(title, img)
            if save:
                cv2.imwrite(title + '_' + time_str + '.png', img)
        cv2.waitKey(1)

    def save_image(self, img, path, title='Image', RGB=True, time_stamp=False):
        """
        Saves the input image using OpenCV.
        Args:
            img (np.array)          : Input image.
            path (str or Path)      : Path to the directory of the image.
            title (str)             : Title of the image.
            time_stamp (bool)       : If true, image name has a time_stamp.
        """
        img = np.reshape(img, (self._height, self._width, -1))
        Path(path).mkdir(parents=True, exist_ok=True)

        if time_stamp:
            time_str = time.strftime("%Y%m%d-%H%M%S")
            title = title + '_' + time_str

        filename = Path(path).joinpath(title + '.png')
        if RGB:
            cv2.imwrite(str(filename), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        else:
            cv2.imwrite(str(filename), (img * 255).astype(np.uint8))

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def view_matrix(self):
        return self._view_mat

    @property
    def projection_matrix(self):
        return self._projection_mat

    @property
    def near(self):
        return self._near

    @property
    def far(self):
        return self._far

    @property
    def fovy(self):
        return self._fovy

    @property
    def aspect(self):
        return self._aspect

    @property
    def camera_up_vector(self):
        return np.array(self._camera_up_vec)

    @property
    def camera_eye_position(self):
        return np.array(self._camera_eye_pos)

    @property
    def camera_target_position(self):
        return np.array(self._camera_target_pos)

class PointCloud:
    """Point cloud
    Wrapper around Open3D PointCloud for easier use.
    """

    def __init__(self):
        self._pcd = open3d.geometry.PointCloud()

    def set_points(self, points, colors=None, estimate_normals=False, camera_location=(0, 0, 0), **kwargs):
        """
        Sets the points and colors of the point cloud.
        Args:
            points (np.array)       : 3D Position of the points in the format [3, N_points] where columns are [X; Y; Z].
            colors (np.array)       : Colors of the points in the format [3, N_ponts] where columns are [R; G; B].
            camera_location (list or np.array)      : Location of the camera of orientation of the surface normals.
            estimate_normals (bool) : If true, estimates point normals.
            **kwargs                : Arguments for estimate_normal method.
        """
        self._pcd.points = open3d.utility.Vector3dVector(points.transpose())
        if colors is not None:
            self._pcd.colors = open3d.utility.Vector3dVector(colors[:3, :].transpose() / 255.)
        if estimate_normals:
            self.estimate_normals(camera_location=camera_location, **kwargs)

    def estimate_normals(self, camera_location, **kwargs):
        """
        Computes the surface normal of the point cloud. It also
        orients the surface normals towards the camera location.
        Args:
            camera_location (list or np.array)      : Location of the camera of orientation of the surface normals.
            **kwargs                                : Arguments for estimate normal method.
        """
        if len(self.points) > 0:
            self._pcd.estimate_normals(**kwargs)
            self._pcd.orient_normals_towards_camera_location(camera_location=camera_location)
            self._pcd.normalize_normals()

    def show(self):
        """
        Shows the pointcloud. Note that this method blocks
        the process. So avoid using it in a loop. If we really
        need the non-blocking visualization, there is a trick to
        make it work:
        http://www.open3d.org/docs/release/tutorial/Advanced/non_blocking_visualization.html
        """
        if len(self.points) > 0:
            open3d.visualization.draw_geometries([self._pcd])

    @property
    def points(self):
        # transpose it to be consistent with Camera class
        return np.asarray(self._pcd.points).transpose()

    @property
    def colors(self):
        # transpose it to be consistent with Camera class
        return np.asarray(self._pcd.colors).transpose()

    @property
    def normals(self):
        # transpose it to be consistent with Camera class
        return np.asarray(self._pcd.normals).transpose()

    @property
    def pcd(self):
        return self._pcd