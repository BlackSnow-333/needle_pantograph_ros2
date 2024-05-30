#!/usr/bin/env python3

import vtk
from vtkmodules.all import * 
import pydicom
import os
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import threading 
from rclpy.executors import MultiThreadedExecutor

# Helper class to format slice status message
class StatusMessage:
    @staticmethod
    def format(slice: int, max_slice: int):
        return f'Slice Number {slice + 1}/{max_slice + 1}'


# Define own interaction style
class MyVtkInteractorStyleImage(vtkInteractorStyleImage):
    def __init__(self, pointPublisher):
        super().__init__()

        # Event observers for user interaction
        self.AddObserver('KeyPressEvent',self.key_press_event)
        self.AddObserver('MouseWheelForwardEvent', self.mouse_wheel_forward_event)
        self.AddObserver('MouseWheelBackwardEvent', self.mouse_wheel_backward_event)
        self.AddObserver('LeftButtonPressEvent', self.on_left_button_down)
        self.AddObserver('MouseMoveEvent',self.on_mouse_move)

        # Initialization
        self.image_viewer = None
        self.status_mapper = None
        self.folder_path = None
        self.dicom_images = []
        self.dicom_files = []
        self.slice = 0
        self.min_slice = 0
        self.max_slice = 0
        self.space_coords = []
        self.image_dims = [1,1]
        self.pointPublisher = pointPublisher

        # Inialize transform for image rotation
        self.transform = vtk.vtkTransform()

        # Create a custom cursor
        self.CustomCursor = vtk.vtkCursor2D()
        self.CustomCursor.SetModelBounds(-10, 10, -10, 10, -10, 10)
        self.CustomCursor.SetRadius(1)  # Adjust the radius of the cross cursor
        self.CustomCursor.AllOn()
        self.CustomCursor.AxesOn()
        self.CustomCursor.SetFocalPoint(5.0, 5.0, 5.0)
        self.CustomCursor.OutlineOn()
        self.CustomCursor.Update()

        # Create a mapper for the cursor
        self.CursorMapper = vtk.vtkPolyDataMapper2D()
        self.CursorMapper.SetInputConnection(self.CustomCursor.GetOutputPort())

        # Create an actor to display the cursor
        self.CursorActor = vtk.vtkActor2D()
        self.CursorActor.GetProperty().SetColor(1.0, 0.0, 0.0)
        self.CursorActor.SetMapper(self.CursorMapper)

        #Cursor initialization
        self.LastCursorPosition = (0, 0)
        

    def set_image_viewer(self, image_viewer,n_slices):
        self.image_viewer = image_viewer
        self.min_slice = 0
        self.max_slice = n_slices
        self.slice = self.min_slice
        # print(f'Slicer: Min = {self.min_slice}, Max= {self.max_slice}')

    def set_status_mapper(self, status_mapper):
        self.status_mapper = status_mapper
    
    def set_dicom_images(self, folder_path):
        # Read all DICOM images in a folder
        self.folder_path = folder_path
        files = os.listdir(folder_path)
        unsorted_files = [f for f in files if f.endswith('.dcm')]
        self.dicom_files = sorted(unsorted_files)
        # print('Using images',self.dicom_files)

        for filename in self.dicom_files:
            filepath = os.path.join(self.folder_path, filename)
            reader = vtk.vtkDICOMImageReader()
            reader.SetFileName(filepath)
            reader.Update()
            self.dicom_images.append(reader.GetOutput())
        
        self.image_dims = [reader.GetWidth(),reader.GetHeight()]

        self.slice = 0
        self.update_image()

    def move_slice_forward(self):
        if self.slice < self.max_slice-1:
            self.slice = (self.slice + 1) % len(self.dicom_images)
            # Update GUI info
            msg = StatusMessage.format(self.slice, self.max_slice-1)
            self.status_mapper.SetInput(msg)
            self.update_image()
    

    def move_slice_backward(self):
        if self.slice > self.min_slice:
            self.slice = (self.slice - 1) % len(self.dicom_images)
            # Update GUI info
            msg = StatusMessage.format(self.slice, self.max_slice-1)
            self.status_mapper.SetInput(msg)
            self.update_image()

    def key_press_event(self, obj, event):
        key = self.GetInteractor().GetKeySym()
        if key == 'Up':
            self.move_slice_forward()
        elif key == 'Down':
            self.move_slice_backward()
        elif key == "m":
            print("current slice :", self.slice)
            metadata = self.get_metadata(self.slice)
            print(metadata)
        elif key == 's':
            print('Displaying sagittal view')
            self.transform.RotateY(-90)
            self.transform.RotateZ(90)
            self.update_image()
        elif key == 'f':
            print('Displaying frontal view')
            self.transform.RotateX(-90)
            self.update_image()
        elif key == 'r':
            print('rotate left')
            self.transform.RotateY(180)
            self.update_image()


    def mouse_wheel_forward_event(self, obj, event):
        # Cycle forward through images on mouse wheel forward
        self.move_slice_forward()

    def mouse_wheel_backward_event(self, obj, event):
        # Cycle backward through images on mouse wheel forward
        self.move_slice_backward()

    def on_left_button_down(self, obj, event):
        # Mouse pixel coords in window frame
        click_pos =  self.GetInteractor().GetEventPosition() 
        # print("Mouse click coordinates in window frame (x, y) :", click_pos)

        # Convert pixel coords in window frame to pixel coords in image frame
        image_origin = self.get_upper_left_corner_pixel(self.image_viewer)
        # print('Image origin in window frame (x,y) :'+'\n',image_origin)
        pixel_coords = np.array([click_pos[0]-image_origin[0],click_pos[1]-image_origin[1],0,1])
        pixel_coords.shape = (4,1)
        print('Pixel coords in image frame (x,y) : '+'\n', pixel_coords[0],pixel_coords[1])

        # Get metadata of the current image
        metadata = self.get_metadata(self.slice)
        # Origin of the current image (in DICOM base frame)
        IPP = metadata['ImagePositionPatient']
        # Orientation of the current image (in DICOM base frame)
        IOP = metadata['ImageOrientationPatient'] 
        # Pixel spacing values
        deltai = metadata['PixelSpacing'][0]
        deltaj = metadata['PixelSpacing'][1]

        # Transformation matrix from 2D pixels to 3D voxel in base frame
        M1 = deltai * np.array([IOP[0],IOP[1],IOP[2],0])
        M1.shape = (4,1)
        M2 = deltaj * np.array([IOP[3],IOP[4],IOP[5],0])
        M2.shape = (4,1)
        M3 = np.zeros((4,1))
        M4 = np.array([IPP[0],IPP[1],IPP[2],1])
        M4.shape = (4,1)
        M = np.hstack((M1,M2,M3,M4))
        # print('transformation matrix : '+'\n',M)

        # Get homogeneous coords in base frame (in mm)
        self.space_coords = np.matmul(M,pixel_coords)
        np.reshape(self.space_coords,(4,1))
        print('Mouse coords in DICOM base frame : '+'\n',self.space_coords[0], 
              self.space_coords[1], self.space_coords[2])
        
        # Set target point coords for pointPublisher 
        self.pointPublisher.set_target_point(self.space_coords)
        # Publish target point 
        self.pointPublisher.point_publisher_callback()

        return 
    
    def on_mouse_move(self, obj, event):
        # Get the interactor and renderer
        interactor = self.GetInteractor()
        renderer = interactor.GetRenderWindow().GetRenderers().GetFirstRenderer()
        
        # Get mouse position
        x, y = interactor.GetEventPosition()
        
        # Set the cursor bounds
        window_size = interactor.GetRenderWindow().GetSize()
        # self.image_dims = [256,256]
        scale_factor = 2.6953125 # TODO check why is scale factor needed
        dims = [scale_factor * self.image_dims[0], scale_factor * self.image_dims[1]] 

        ofx = (window_size[0] - dims[0])/2 # offset along x axis
        ofy = (window_size[1] - dims[1])/2 #offset along y axis
        self.CustomCursor.SetModelBounds(ofx, dims[0] + ofx, 
                                         ofy, dims[1] + ofy,
                                         -0.5, 0.5)
        
        # Check if cursor position has changed
        if (x, y) != self.LastCursorPosition:
            self.LastCursorPosition = (x, y)
            self.CustomCursor.SetFocalPoint(x, y, 0.0)
            self.CustomCursor.Update()
            
            # Render the updated cursor
            renderer.AddActor(self.CursorActor)
            interactor.Render()
            # print('Cursor position : ',self.LastCursorPosition)

    def update_image(self):
        # Update displayed image based on current image index
        image_data = self.dicom_images[self.slice]
        
        # Apply transform to reslice image for current view
        reslice = vtk.vtkImageReslice()
        reslice.SetInputData(image_data)
        reslice.SetResliceTransform(self.transform)
        reslice.SetInterpolationModeToLinear()
        reslice.Update()

        self.image_viewer.SetInputData(reslice.GetOutput())
        self.image_viewer.Render()

    def get_metadata(self,n_slice):
        # Retrieve metadata of the current DICOM image
        current_image_path = os.path.join(self.folder_path, self.dicom_files[n_slice])
        ds = pydicom.dcmread(current_image_path)  # Read metadata from the file
        # Extract relevant metadata
        # print(ds) # show all DICOM fields
        metadata = {
            'PatientName': ds.PatientName,
            'StudyDescription': ds.StudyDescription,
            'Rows': ds.Rows,
            'Columns': ds.Columns,
            'PixelSpacing': ds.PixelSpacing,
            'Frame of Reference UID' : ds.FrameOfReferenceUID,
            'ImageOrientationPatient': ds.ImageOrientationPatient,
            'ImagePositionPatient': ds.ImagePositionPatient,
            'SliceThickness': ds.SliceThickness  
            # Add more fields as needed
        }
        return metadata
    
    def get_upper_left_corner_pixel(self, viewer):
        # Get the renderer used by vtkImageViewer2
        renderer = viewer.GetRenderer()

        # Get image data
        image_data = viewer.GetInput()
        dims = image_data.GetDimensions()
        spacing = image_data.GetSpacing()

        # Calculate the center of the image in world coordinates
        center_world = image_data.GetCenter()

        # Calculate the offset from the center to the upper left corner in world coordinates
        ul_corner_world = [center_world[0] - dims[0] * spacing[0] / 2,
                        center_world[1] + dims[1] * spacing[1] / 2,
                        center_world[2]]

        # Transform from world coordinates to display coordinates
        renderer.SetWorldPoint(ul_corner_world[0], ul_corner_world[1], ul_corner_world[2], 1.0)
        renderer.WorldToDisplay()
        ul_corner_display = renderer.GetDisplayPoint()

        # Convert display coordinates to pixel coordinates
        ul_pixel = [int(ul_corner_display[0]), int(ul_corner_display[1])]
        return ul_pixel

def read_dicom_images(folder_path):
    # Read all DICOM images in a folder
    files = os.listdir(folder_path)
    dicom_files = [f for f in files if f.endswith('.dcm')]

    dicom_images = []
    for filename in dicom_files:
        filepath = os.path.join(folder_path, filename)
        reader = vtk.vtkDICOMImageReader()
        reader.SetFileName(filepath)
        reader.Update()
        dicom_images.append(reader.GetOutput())

    return dicom_images

def start_gui(folder_path, my_interactor_style):
    # Thread info (testing only)
    print("gui assigned to thread : {}".format(threading.current_thread().name))
    print("ID of process running gui: {}".format(os.getpid()))

    # Read DICOM images
    dicom_images = read_dicom_images(folder_path)

    # Create VTK image viewer
    image_viewer = vtk.vtkImageViewer2()
 
    # Create slice status message
    slice_text_prop = vtkTextProperty()
    slice_text_prop.SetFontFamilyToCourier()
    slice_text_prop.SetFontSize(20)
    slice_text_prop.SetVerticalJustificationToBottom()
    slice_text_prop.SetJustificationToLeft()

    slice_text_mapper = vtkTextMapper()
    msg = StatusMessage.format(0,len(dicom_images)-1)
    slice_text_mapper.SetInput(msg)
    slice_text_mapper.SetTextProperty(slice_text_prop)

    slice_text_actor = vtkActor2D()
    slice_text_actor.SetMapper(slice_text_mapper)
    slice_text_actor.SetPosition(10, 10)

    # Create usage hint message
    usage_text_prop = vtkTextProperty()
    usage_text_prop.SetFontFamilyToCourier()
    usage_text_prop.SetFontSize(14)
    usage_text_prop.SetVerticalJustificationToTop()
    usage_text_prop.SetJustificationToLeft()
    usage_text_mapper = vtkTextMapper()
    usage_text_mapper.SetInput(
        'Slice with mouse wheel\n  or Up/Down-Key\n press m for current slice metadata'
    )
    usage_text_mapper.SetTextProperty(usage_text_prop)

    usage_text_actor = vtkActor2D()
    usage_text_actor.SetMapper(usage_text_mapper)
    usage_text_actor.GetPositionCoordinate().SetCoordinateSystemToNormalizedDisplay()
    usage_text_actor.GetPositionCoordinate().SetValue(0.02, 1) 

    # Set up render window
    render_window = vtk.vtkRenderWindow()
    image_viewer.SetRenderWindow(render_window)

    # Set up interactor
    # Create an interactor with our own style (inherit from
    # vtkInteractorStyleImage in order to catch mousewheel and key events.
    render_window_interactor = vtkRenderWindowInteractor()

    # Make imageviewer2 and sliceTextMapper visible to our interactorstyle
    # to enable slice status message updates when  scrolling through the slices.
    my_interactor_style.set_image_viewer(image_viewer,len(dicom_images))
    my_interactor_style.set_status_mapper(slice_text_mapper)

    # Make the interactor use our own interactor style
    image_viewer.SetupInteractor(render_window_interactor)
    render_window_interactor.SetInteractorStyle(my_interactor_style)
    render_window_interactor.Render()

    # Add slice status message and usage hint message to the renderer.
    image_viewer.GetRenderer().AddActor2D(slice_text_actor)
    image_viewer.GetRenderer().AddActor2D(usage_text_actor)

    # Add custom cursor
    image_viewer.GetRenderer().AddActor2D(my_interactor_style.CursorActor)
    my_interactor_style.CustomCursor.Update()

    # Set DICOM images in the custom interactor
    render_window_interactor.GetInteractorStyle().set_dicom_images(folder_path)

    # Display the first DICOM image
    render_window_interactor.GetInteractorStyle().update_image()

    # Initialize rendering and interaction.
    # image_viewer.GetRenderer().ResetCamera()
    # Window parameters
    colors = vtkNamedColors()
    image_viewer.GetRenderer().SetBackground(colors.GetColor3d('SlateGray'))
    image_viewer.GetRenderWindow().SetSize(800, 800)
    image_viewer.GetRenderWindow().SetWindowName('DICOM Reader')
    image_viewer.Render()
    image_viewer.GetRenderWindow().GetInteractor().Start()

    

class pointPublisher(Node):
    # Publishes a point 3D position as a Float64MultiArray

    def __init__(self):
        super().__init__('point_publisher')
        self.target_point = [0.0, 0.0, 0.0]
        self.publisher_ = self.create_publisher(Float64MultiArray, 'target_point', 10)
        # timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.point_publisher_callback)

    def point_publisher_callback(self):
        msg = Float64MultiArray()
        # Point offsets for testing only
        offset = [100,100,50]
        
        if len(self.target_point) == 0 :
            msg.data = [0.0, 0.0, 0.0] 
        else : 
            msg.data.append(np.squeeze(self.target_point[0]) + offset[0])
            msg.data.append(np.squeeze(self.target_point[1]) + offset[1])
            msg.data.append(np.squeeze(self.target_point[2]) + offset[2])
        
        # Publish the message
        self.publisher_.publish(msg)  
        self.get_logger().info('Publishing target_point : %d, %d, %d' % (msg.data[0],msg.data[1],msg.data[2]))    

    def set_target_point(self,coords) :
        self.target_point = coords

def run_ros2_node(pointPublisher):
    print("ros2 node assigned to thread : {}".format(threading.current_thread().name))
    print("ID of process running ros2 node: {}".format(os.getpid()))
    while rclpy.ok():
        rclpy.spin_once(pointPublisher, timeout_sec=0.1)
    rclpy.shutdown()



def main(args=None):
    # print('VTK Version:', vtkVersion.GetVTKVersion())

    # Path to the folder containing DICOM images
    folder_path = "/home/telecom/dev/ws_pantograph_ros2/src/needle_pantograph_ros2/DicomTestImages/matlab/examples/sample_data/DICOM/digest_article"
    
    # ROS2 node initialization
    rclpy.init(args=args)
    point_publisher = pointPublisher()

    # gui initialization
    gui = MyVtkInteractorStyleImage(point_publisher)

    # Create ros node in a separate thread 
    t_ros = threading.Thread(target= run_ros2_node,args= (point_publisher,))
    t_ros.start()  # Start thread

    # Launch DICOM Reader in the main thread
    start_gui(folder_path,gui)

    # Joint threads once the execution is finished
    t_ros.join()


if __name__ == '__main__':
    main()