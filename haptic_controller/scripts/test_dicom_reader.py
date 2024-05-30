import vtk
import threading
import os

# Step 1: Read DICOM images using vtkDICOMImageReader
def read_dicom_series(directory):
    reader = vtk.vtkDICOMImageReader()
    reader.SetDirectoryName(directory)
    reader.Update()
    return reader

def read_dicom_images(folder_path):
    dicom_images = []
    # Read all DICOM images in a folder
    files = os.listdir(folder_path)
    unsorted_files = [f for f in files if f.endswith('.dcm')]
    dicom_files = sorted(unsorted_files)
    
    for filename in dicom_files:
        filepath = os.path.join(folder_path, filename)
        reader = vtk.vtkDICOMImageReader()
        reader.SetFileName(filepath)
        reader.Update()
        dicom_images.append(reader.GetOutput())
    
    return dicom_images

# Step 2: Configure vtkImageReslice with vtkTransform to compute the desired views
def reslice_volume(vtk_image, orientation):
    transform = vtk.vtkTransform()
    
    if orientation == 'sagittal':
        # XZ plane (sagittal)
        transform.RotateWXYZ(90, 0, 1, 0)  # Rotate around Y axis
    elif orientation == 'frontal':
        # YZ plane (frontal)
        transform.RotateWXYZ(90, 1, 0, 0)  # Rotate around X axis
    else:
        raise ValueError("Unknown orientation")
    
    reslice = vtk.vtkImageReslice()
    reslice.SetInputConnection(vtk_image.GetOutputPort())
    reslice.SetResliceTransform(transform)
    reslice.SetInterpolationModeToLinear()
    reslice.SetOutputDimensionality(2)
    
    # Reslice axes for sagittal and frontal views
    if orientation == 'sagittal':
        reslice.SetResliceAxesDirectionCosines(0, 0, -1, 0, 1, 0, 1, 0, 0)
    elif orientation == 'frontal':
        reslice.SetResliceAxesDirectionCosines(1, 0, 0, 0, 0, -1, 0, 1, 0)
    
    reslice.Update()
    return reslice.GetOutput()

# Step 3: Display the computed views using VTK's image viewer
def display_image(vtk_image, window_name):
    viewer = vtk.vtkImageViewer2()
    viewer.SetInputData(vtk_image)
    render_window = vtk.vtkRenderWindow()
    render_window.SetWindowName(window_name)
    viewer.SetRenderWindow(render_window)
    interactor = vtk.vtkRenderWindowInteractor()
    viewer.SetupInteractor(interactor)
    viewer.Render()
    viewer.GetRenderer().ResetCamera()
    viewer.Render()
    interactor.Initialize()
    interactor.Start()

# Function to run the display in a separate thread
def display_in_thread(vtk_image, window_name):
    thread = threading.Thread(target=display_image, args=(vtk_image, window_name))
    thread.start()
    return thread

# Main function
def main():
    directory = '/home/telecom/dev/ws_pantograph_ros2/src/needle_pantograph_ros2/DicomTestImages/matlab/examples/sample_data/DICOM/digest_article'
    reader = read_dicom_series(directory)

    sagittal_image = reslice_volume(reader, 'sagittal')
    frontal_image = reslice_volume(reader, 'frontal')
    
    print("Displaying original image...")
    # thread1 = display_in_thread(reader.GetOutput(), "Original View")
    t1 = threading.Thread(target=display_image, args=(reader.GetOutput(), 'Original View'))
    t1.start()
    print("Window 1 assigned to thread : {}".format(threading.current_thread().name))
    print("ID of process running window 1 : {}".format(os.getpid()))

    print("Displaying sagittal image...")
    # thread2 = display_in_thread(sagittal_image, "Sagittal View")
    t2 = threading.Thread(target=display_image, args=(sagittal_image, 'Sagittal View'))
    t2.start()
    print("Window 2 assigned to thread : {}".format(threading.current_thread().name))
    print("ID of process running window 2 : {}".format(os.getpid()))

    print("Displaying frontal image...")
    # thread3 = display_in_thread(frontal_image, "Frontal View")
    t3 = threading.Thread(target=display_image, args=(frontal_image, 'Frontal View'))
    t3.start()
    print("Window 3 assigned to thread : {}".format(threading.current_thread().name))
    print("ID of process running window 3 : {}".format(os.getpid()))

    # Join threads to ensure the main program waits for all windows to be closed
    t1.join()
    t2.join()
    t3.join()

if __name__ == '__main__':
    main()
