import sys
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
import os
import cv2
import dicom_viewer.threeD.loaddicomfile as ldf
import numpy as np
from scipy.spatial.transform import Rotation as R
from dicom_viewer.threeD.vol_view_module import C3dView

from PyQt5.QtCore import pyqtSlot

import rclpy

# adding folder dicom_viewer to the system path
sys.path.insert(0,'/home/telecom/dev/ws_pantograph_ros2/src/needle_pantograph_ros2/dicom_viewer')
from dicom_viewer.pointPublisher import PointPublisher

class CthreeD(QDialog):
    def __init__(self):
        super().__init__()
        # path = os.getcwd()
        path = '/home/telecom/dev/ws_pantograph_ros2/src/needle_pantograph_ros2/dicom_viewer/dicom_viewer/threeD'
        os.chdir(path)
        self.directory = os.getcwd()
        loadUi('threeD_module.ui', self)
        self.setWindowTitle('3D Processing')
        self.image = None
        self.voxel = None
        self.processedvoxel = None
        self.v1, self.v2, self.v3 = None, None, None
        self.volWindow = None
        self.dicomButton.clicked.connect(self.dicom_clicked)
        self.axial_hSlider.valueChanged.connect(self.updateimg)
        self.axial_vSlider.valueChanged.connect(self.updateimg)
        self.sagittal_hSlider.valueChanged.connect(self.updateimg)
        self.sagittal_vSlider.valueChanged.connect(self.updateimg)
        self.coronal_hSlider.valueChanged.connect(self.updateimg)
        self.coronal_vSlider.valueChanged.connect(self.updateimg)
        self.colormap = None
        # 這樣可以把"被activate"的Item轉成str傳入connect的function（也可以用int之類的，會被enum）
        self.colormapBox.activated[str].connect(self.colormap_choice)
        self.colormapDict = {'GRAY': None,
                             'AUTUMN': cv2.COLORMAP_AUTUMN,
                             'BONE': cv2.COLORMAP_BONE,
                             'COOL': cv2.COLORMAP_COOL,
                             'HOT': cv2.COLORMAP_HOT,
                             'HSV': cv2.COLORMAP_HSV,
                             'JET': cv2.COLORMAP_JET,
                             'OCEAN': cv2.COLORMAP_OCEAN,
                             'PINK': cv2.COLORMAP_PINK,
                             'RAINBOW': cv2.COLORMAP_RAINBOW,
                             'SPRING': cv2.COLORMAP_SPRING,
                             'SUMMER': cv2.COLORMAP_SUMMER,
                             'WINTER': cv2.COLORMAP_WINTER
                             }
        self.volButton.clicked.connect(self.open_3dview)

        self.w, self.h = self.imgLabel_1.width(), self.imgLabel_1.height()

        self.imgLabel_1.type = 'axial'
        self.imgLabel_2.type = 'sagittal'
        self.imgLabel_3.type = 'coronal'

        self.axialGrid.setSpacing(0)
        self.saggitalGrid.setSpacing(0)
        self.coronalGrid.setSpacing(0)

        h = QSpacerItem(10, 10, QSizePolicy.Fixed, QSizePolicy.Fixed)
        v = QSpacerItem(10, 10, QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.axial_vBox.setSpacing(0)
        self.axial_vBox.insertSpacerItem(0, v)
        self.axial_vBox.insertSpacerItem(2, v)
        self.axial_hBox.setSpacing(0)
        self.axial_hBox.insertSpacerItem(0, h)
        self.axial_hBox.insertSpacerItem(2, h)
        self.saggital_vBox.setSpacing(0)
        self.saggital_vBox.insertSpacerItem(0, v)
        self.saggital_vBox.insertSpacerItem(2, v)
        self.saggital_hBox.setSpacing(0)
        self.saggital_hBox.insertSpacerItem(0, h)
        self.saggital_hBox.insertSpacerItem(2, h)
        self.coronal_vBox.setSpacing(0)
        self.coronal_vBox.insertSpacerItem(0, v)
        self.coronal_vBox.insertSpacerItem(2, v)
        self.coronal_hBox.setSpacing(0)
        self.coronal_hBox.insertSpacerItem(0, h)
        self.coronal_hBox.insertSpacerItem(2, h)

        self.colormap_hBox.insertStretch(2)
        self.colormap_hBox.insertSpacerItem(0, QSpacerItem(30, 0, QSizePolicy.Fixed,  QSizePolicy.Fixed))

        self.savesliceButton.clicked.connect(self.saveslice_clicked)
        self.dcmInfo = None
        self.imgLabel_1.mpsignal.connect(self.cross_center_mouse)
        self.imgLabel_2.mpsignal.connect(self.cross_center_mouse)
        self.imgLabel_3.mpsignal.connect(self.cross_center_mouse)

        self.cross_recalc = True
        self.savenpyButton.clicked.connect(self.save_npy_clicked)
        self.loadnpyButton.clicked.connect(self.load_npy_clicked)
        self.downscaled = 2
        self.dsampleButton.clicked.connect(self.downsample)

        # Get mouse coords from axial view
        self.imgLabel_1.coords_updated.connect(self.handle_mouse_coords_update)

        # Get slices metadata
        self.all_metadata = []

        # Button setup for choosing insertion and target point
        self.SetInsertionPointButton.clicked.connect(self.set_insertion_pt_clicked)
        self.SetTargetPointButton.clicked.connect(self.set_target_pt_clicked) 

        # Insertion and target point coords
        self.insertion_point = [0.0, 0.0, 0.0]
        self.target_point = [0.0, 0.0, 0.0]

        self.T_WP = np.eye(4)
        self.T_PI = np.eye(4)

        # Scale factor (testing only)
        self.scale_factor = 0.001     

        # Test ROS2 publisher for point coords
        self.insertion_point_msg = [0.0, 0.0, 0.0]
        self.target_point_msg = [0.0, 0.0, 0.0]
        
        rclpy.init(args=None)
        self.point_publisher_node = PointPublisher()
        print('Point publisher created')

    def downsample(self):
        self.processedvoxel = self.processedvoxel[::self.downscaled, ::self.downscaled, ::self.downscaled]
        self.update_shape()
        self.updateimg()

    def save_npy_clicked(self):
        fname, _filter = QFileDialog.getSaveFileName(self, 'save file', '~/untitled', "Image Files (*.npy)")
        if fname:
            np.save(fname, self.processedvoxel)
        else:
            print('Error')

    def load_npy_clicked(self):
        fname, _filter = QFileDialog.getOpenFileName(self, 'open file', '~/Desktop', "Image Files (*.NPY *.npy)")
        self.processedvoxel = np.load(fname)
        self.update_shape()
        self.savetemp()
        self.updateimg()

    def set_directory(self):
        os.chdir(self.directory)

    def cross_center_mouse(self, _type):
        self.cross_recalc = False
        if _type == 'axial':
            self.axial_hSlider.setValue(int(self.imgLabel_1.crosscenter[0] *
                                        self.axial_hSlider.maximum() / self.imgLabel_1.width()))
            self.axial_vSlider.setValue(int(self.imgLabel_1.crosscenter[1] *
                                        self.axial_vSlider.maximum() / self.imgLabel_1.height()))
        elif _type == 'sagittal':
            self.sagittal_hSlider.setValue(int(self.imgLabel_2.crosscenter[0] *
                                           self.sagittal_hSlider.maximum() / self.imgLabel_2.width()))
            self.sagittal_vSlider.setValue(int(self.imgLabel_2.crosscenter[1] *
                                           self.sagittal_vSlider.maximum() / self.imgLabel_2.height()))
        elif _type == 'coronal':
            self.coronal_hSlider.setValue(int(self.imgLabel_3.crosscenter[0] *
                                          self.coronal_hSlider.maximum() / self.imgLabel_3.width()))
            self.coronal_vSlider.setValue(int(self.imgLabel_3.crosscenter[1] *
                                          self.coronal_vSlider.maximum() / self.imgLabel_3.height()))
        else:
            pass

        self.imgLabel_1.crosscenter = [
            int(self.axial_hSlider.value() * self.imgLabel_1.width() / self.axial_hSlider.maximum()),
            int(self.axial_vSlider.value() * self.imgLabel_1.height() / self.axial_vSlider.maximum())]
        self.imgLabel_2.crosscenter = [
            int(self.sagittal_hSlider.value() * self.imgLabel_2.width() / self.sagittal_hSlider.maximum()),
            int(self.sagittal_vSlider.value() * self.imgLabel_2.height() / self.sagittal_vSlider.maximum())]
        self.imgLabel_3.crosscenter = [
            int(self.coronal_hSlider.value() * self.imgLabel_3.width() / self.coronal_hSlider.maximum()),
            int(self.coronal_vSlider.value() * self.imgLabel_3.height() / self.coronal_vSlider.maximum())]
        self.updateimg()

        self.cross_recalc = True

    def saveslice_clicked(self):
        fname, _filter = QFileDialog.getSaveFileName(self, 'save file', '~/untitled', "Image Files (*.jpg)")
        if fname:
            if self.savesliceBox.currentText() == 'Axial':
                cv2.imwrite(fname, self.imgLabel_1.processedImage)
            elif self.savesliceBox.currentText() == 'Saggital':
                cv2.imwrite(fname, self.imgLabel_2.processedImage)
            elif self.savesliceBox.currentText() == 'Coronal':
                cv2.imwrite(fname, self.imgLabel_3.processedImage)
            else:
                print('No slice be chosen')
        else:
            print('Error')
        pass

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.w = self.imgLabel_1.width()
        self.h = self.imgLabel_1.height()
        if self.processedvoxel is not None:
            self.updateimg()

    def open_3dview(self):
        self.volWindow.setWindowTitle('3D View')
        self.volWindow.vol_show()
        self.volWindow.show()

    def colormap_choice(self, text):
        self.colormap = self.colormapDict[text]
        self.updateimg()

    def dicom_clicked(self):
        dname = QFileDialog.getExistingDirectory(self, 'choose dicom directory')
        print(dname)
        self.load_dicomfile(dname)
    
    def set_insertion_pt_clicked(self):
        self.insertion_point = self.imgLabel_1.slice_loc
        print('Insertion point coords in image frame : [x, y, z] \n', self.insertion_point)
        
        # Get slice metadata
        current_slice_number = self.insertion_point[2]
        slice_metadata = self.all_metadata[current_slice_number]
        # print('Slice metadata : \n', self.all_metadata[coords[2]])

        # Convert pixel coords to homogeneous pixel coords
        pixel_coords = [self.insertion_point[0], self.insertion_point[1], 0, 1]

        # Insertion point coords in patient frame (in mm)
        patient_insertion_point = self.image_to_patient(pixel_coords,slice_metadata)
        
        # Apply scale factor to patient coords to transform from mm to m 
        patient_insertion_point[:3] = self.scale_factor * patient_insertion_point[:3]  
    
        # Robot insertion point coords in world frame (in m)
        robot_insertion_point = [0.0, 0.16056, 0.09]  
        
        # Compute translation from world frame to patient frame (in m)
        # Needed to overlap patient_insertion_point and robot_insertion_point
        t_WP = robot_insertion_point - patient_insertion_point[:3]

        # Compute rotation needed to orientate image as desired
        # R_WP = np.eye(3)
        R_WP = R.from_euler('xyz', [0,0,0], degrees=True).as_matrix()

        # Compute hogeneous transformation matrix from world frame to patient frame
        # offset = [-0.002719, 0.0, 0.0]  # offset for testing only
        self.T_WP = np.eye(4)
        self.T_WP[:3, :3] = R_WP
        self.T_WP[:3, 3] = t_WP  # + offset

        # Verify the transformation
        # T_WP in m T_PI in mm
        p1 = np.matmul(self.T_PI,pixel_coords).ravel()
        p1[:3] = self.scale_factor * p1[:3]  # transform coords in mm to m 
        self.insertion_point_msg = (np.matmul(self.T_WP,p1).ravel()) 
        print('Insertion point updated : \n', self.insertion_point_msg[:3])
        
        # Publish coords to ROS2 topic
        self.point_publisher_node.publish_points(self.insertion_point_msg[:3], self.insertion_point_msg[:3])

        # TODO Draw point marker at insertion point
    
    def set_target_pt_clicked(self):
        self.target_point = self.imgLabel_1.slice_loc
        print('Target point pixel coords in image frame : [x, y, z]  \n', self.target_point)

        # Get slice metadata
        current_slice_number = self.target_point[2]
        slice_metadata = self.all_metadata[current_slice_number]
        # print('Slice metadata : \n', self.all_metadata[coords[2]])

        # Transformation from image to world frame
        # Convert pixel coords to homogeneous pixel coords
        pixel_coords = [self.target_point[0], self.target_point[1], 0, 1]
        
        # Target point coords in patient frame (in mm)
        patient_target_point = self.image_to_patient(pixel_coords,slice_metadata)
        
        # Apply scale factor to patient coords to transform from mm to m 
        patient_target_point[:3] = self.scale_factor * patient_target_point[:3]

        # Target point coords in world frame (in m)
        world_target_point = np.matmul(self.T_WP, patient_target_point).ravel() 
        
        self.target_point_msg = world_target_point
        print('Target point 3D coords in robot world frame : [x,y,z] \n', self.target_point_msg[:3])

        # Publish coords to ROS2 topic
        self.point_publisher_node.publish_points(self.insertion_point_msg[:3], self.target_point_msg[:3])

        # TODO Draw point marker at insertion point


    def load_dicomfile(self, dname):
        self.dcmList.clear()
        patient, self.all_metadata = ldf.load_scan(dname)
        imgs = ldf.get_pixels_hu(patient)
        self.voxel = self.linear_convert(imgs)
        self.processedvoxel = self.voxel.copy()

        self.update_shape()

        self.imgLabel_1.setMouseTracking(True)
        self.imgLabel_2.setMouseTracking(True)
        self.imgLabel_3.setMouseTracking(True)

        self.updateimg()
        self.set_directory()
        self.volWindow = C3dView()
        self.volWindow.imgs = imgs
        self.volWindow.patient = patient
        self.dcmInfo = ldf.load_dcm_info(dname, self.privatecheckBox.isChecked())
        self.updatelist()

    def update_shape(self):
        self.v1, self.v2, self.v3 = self.processedvoxel.shape
        self.sagittal_vSlider.setMaximum(self.v1-1)
        self.coronal_vSlider.setMaximum(self.v1-1)
        self.sagittal_hSlider.setMaximum(self.v2-1)
        self.axial_vSlider.setMaximum(self.v2-1)
        self.coronal_hSlider.setMaximum(self.v3-1)
        self.axial_hSlider.setMaximum(self.v3-1)
        self.sagittal_vSlider.setValue(self.sagittal_vSlider.maximum()//2)
        self.coronal_vSlider.setValue(self.coronal_vSlider.maximum()//2)
        self.sagittal_hSlider.setValue(self.sagittal_hSlider.maximum()//2)
        self.axial_vSlider.setValue(self.axial_vSlider.maximum()//2)
        self.coronal_hSlider.setValue(self.coronal_hSlider.maximum()//2)
        self.axial_hSlider.setValue(self.axial_hSlider.maximum()//2)

    def updatelist(self):
        for item in self.dcmInfo:
            # 單純字串的話，可以不需要QListWidgetItem包裝也沒關係
            self.dcmList.addItem(QListWidgetItem('%-20s\t:  %s' % (item[0], item[1])))

    def updateimg(self):
        a_loc = self.sagittal_vSlider.value()
        c_loc = self.axial_vSlider.value()
        s_loc = self.axial_hSlider.value()

        axial = (self.processedvoxel[a_loc, :, :]).astype(np.uint8).copy()
        sagittal = (self.processedvoxel[:, :, s_loc]).astype(np.uint8).copy()
        coronal = (self.processedvoxel[:, c_loc, :]).astype(np.uint8).copy()

        self.imgLabel_1.slice_loc = [s_loc, c_loc, a_loc]
        self.imgLabel_2.slice_loc = [s_loc, c_loc, a_loc]
        self.imgLabel_3.slice_loc = [s_loc, c_loc, a_loc]

        if self.cross_recalc:
            self.imgLabel_1.crosscenter = [self.w*s_loc//self.v3, self.h*c_loc//self.v2]
            self.imgLabel_2.crosscenter = [self.w*c_loc//self.v2, self.h*a_loc//self.v1]
            self.imgLabel_3.crosscenter = [self.w*s_loc//self.v3, self.h*a_loc//self.v1]

        if self.colormap is None:
            self.imgLabel_1.processedImage = axial
            self.imgLabel_2.processedImage = sagittal
            self.imgLabel_3.processedImage = coronal
        else:
            self.imgLabel_1.processedImage = cv2.applyColorMap(axial, self.colormap)
            self.imgLabel_2.processedImage = cv2.applyColorMap(sagittal, self.colormap)
            self.imgLabel_3.processedImage = cv2.applyColorMap(coronal, self.colormap)

        self.imgLabel_1.display_image(1)
        self.imgLabel_2.display_image(1)
        self.imgLabel_3.display_image(1)

    def image_to_patient(self, pixel_coords, slice_metadata):
        # Get metadata of the current image
        # Origin of the current image (in Patient frame)
        IPP = slice_metadata['ImagePositionPatient']
        # Orientation of the current image (in Patient frame)
        IOP = slice_metadata['ImageOrientationPatient']
        # Pixel spacing values
        deltai = slice_metadata['PixelSpacing'][0]
        deltaj = slice_metadata['PixelSpacing'][1]

        # Transformation matrix from 2D pixels to 3D voxel in base frame
        C1 = deltai * np.array([IOP[0], IOP[1], IOP[2], 0])  # direction cosines of image x axis
        C2 = deltaj * np.array([IOP[3], IOP[4], IOP[5], 0])  # direction cosines of image y axis
        C3 = np.array([0, 0, 0, 0])
        C4 = np.array([IPP[0], IPP[1], IPP[2], 1])  # Position of the image origin in patient frame
        self.T_PI = np.transpose(np.stack((C1, C2, C3, C4)))  # Transformation matrix from Patient to image frame
        # print('transformation matrix : '+'\n',M)

        # Get homogeneous coords in base frame (in mm)
        patient_coords = np.matmul(self.T_PI, pixel_coords).ravel()
        # Convert to 3D coords
        # space_coords = space_coords[:3]
        return patient_coords

    @pyqtSlot(list)
    def handle_mouse_coords_update(self, coords):
        self.updateimg()
        return
    
    @staticmethod
    def linear_convert(img):
        convert_scale = 255.0 / (np.max(img) - np.min(img))
        converted_img = convert_scale * img - (convert_scale * np.min(img))
        return converted_img

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = CthreeD()
    ex.show()
    sys.exit(app.exec_())
