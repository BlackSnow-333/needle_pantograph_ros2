import numpy as np
import pydicom
import os
import scipy.ndimage
from skimage import measure
import glob

def load_dcm_info(path, private):
    # 取第一個slice就好
    slice_for_info = pydicom.read_file(path + '/' + os.listdir(path)[0], force=True)


    if private:
        name = ('Patient Name', 'Private')
        _id = ('Patient ID', 'Private')
        age = ('Patient Age', 'Private')
        sex = ('Patient Sex', 'Private')
        institution_name = ('Institution', 'Private')
        date = ('Date', 'Private')
        modality = ('Modality', 'Private')
        manufacturer = ('Manufacturer', 'Private')
    else:
        try:
            name = ('Patient Name', str(slice_for_info.PatientName).split('  ', 1)[1])
        except:
            name = ('Patient Name', 'Anonymous')
        try:
            _id = ('Patient ID', str(slice_for_info.PatientID))
        except:
            _id = ('Patient ID', 'Unknown')
        try:
            age = ('Patient Age', str(slice_for_info.PatientAge))
        except:
            age = ('Patient Age', 'Unknown')

        try:
            sex = ('Patient Sex', str(slice_for_info.PatientSex))
        except:
            sex = ('Patient Sex', 'Unknown')

        try:
            institution_name = ('Institution', str(slice_for_info.InstitutionName))
        except:
            institution_name = ('Institution', 'Unknown')

        try:
            date = ('Date', str(slice_for_info.InstanceCreationDate))
        except:
            date = ('Date', 'Unknown')

        try:
            modality = ('Modality', str(slice_for_info.Modality))
        except:
            modality = ('Modality', 'Unknown')

        try:
            manufacturer = ('Manufacturer', str(slice_for_info.Manufacturer))
        except:
            manufacturer = ('Manufacturer', 'Unknown')

        # Test get DICOM metadata

        try:
            IPP = ('Image Position Patient', str(slice_for_info.ImagePositionPatient))
        except:
            IPP = ('Image Position Patient', 'Unknown')

        try:
            IOP_x = ('Image Orientation Patient (x_axis)', str(slice_for_info.ImageOrientationPatient[0:3]))
        except:
            IOP_x = ('Image Orientation Patient (x_axis)', 'Unknown')

        try:
            IOP_y = ('Image Orientation Patient (y_axis)', str(slice_for_info.ImageOrientationPatient[3:6]))
        except:
            IOP_y = ('Image Orientation Patient (y_axis)', 'Unknown')
        
        try:
            slice_thickness = ('Slice thickness', str(slice_for_info.SliceThickness))
        except:
            slice_thickness = ('Slice thickness', 'Unknown')

        try:
            delta_i = ('Pixel spacing delta_i', str(slice_for_info.PixelSpacing[0]))
        except:
            delta_i = ('Pixel spacing delta_i', 'Unknown')
        
        try:
            delta_j = ('Pixel spacing delta_j', str(slice_for_info.PixelSpacing[1]))
        except:
            delta_j = ('Pixel spacing delta_j', 'Unknown')


    info = [name, _id, age, sex, date, institution_name, modality, manufacturer, 
            IPP, IOP_x, IOP_y, slice_thickness, delta_i, delta_j]
    return info


def load_scan(path):
    f = glob.glob(rf'{path}/*.dcm')
    slices = [pydicom.read_file(s, force=True) for s in f]
    # slices = [pydicom.read_file(path + '/' + s, force=True) for s in os.listdir(path)]
    all_metadata = []
    for s in slices:
        s.file_meta.TransferSyntaxUID = pydicom.uid.ImplicitVRLittleEndian
    slices.sort(key=lambda x: int(x.InstanceNumber))
    # thickness在dicom裡面是空白value，算一下給它值（兩個slice間的z差值）
    try:
        slice_thickness = np.abs(slices[0].ImagePositionPatient[2] - slices[1].ImagePositionPatient[2])
    except Exception:
        slice_thickness = np.abs(slices[0].SliceLocation - slices[1].SliceLocation)
    for s in slices:
        s.SliceThickness = slice_thickness

         # Test get DICOM metadata
        slice_metadata = get_metadata(s)
        all_metadata.append(slice_metadata)

    # orientation = slices[0].ImageOrientationPatient
    # print(orientation)

    return slices, all_metadata


def get_pixels_hu(scans):
    image = np.flipud(np.stack([s.pixel_array for s in scans]))
    # Convert to int16 (from sometimes int16),
    # should be possible as values should always be low enough (<32k)
    image = image.astype(np.int16)

    # Set outside-of-scan pixels to 1
    # The intercept is usually -1024, so air is approximately 0
    # 在影像中非人體部位（周遭空氣等）被拍到的地方數值會是-2000，把那些設為0
    image[image == -2000] = 0

    # Convert to Hounsfield units (HU)
    intercept = scans[0].RescaleIntercept
    slope = scans[0].RescaleSlope

    if slope != 1:
        image = slope * image.astype(np.float64)
        image = image.astype(np.int16)

    image += np.int16(intercept)
    return np.array(image, dtype=np.int16)


def resample(image, scan):
    # Determine current pixel spacing
    new_spacing = [1, 1, 1]
    spacing = map(float, ([scan[0].SliceThickness] + [scan[0].PixelSpacing[0]] + [scan[0].PixelSpacing[1]]))
    spacing = np.asarray(list(spacing))

    resize_factor = spacing / new_spacing
    new_real_shape = image.shape * resize_factor
    new_shape = np.round(new_real_shape)
    real_resize_factor = new_shape / image.shape
    new_spacing = spacing / real_resize_factor
    image = scipy.ndimage.interpolation.zoom(image, real_resize_factor)

    return image, new_spacing


def make_mesh(image, threshold=-300, step_size=10):
    print('Transposing surface')
    # 不同的transpose，圖的方位不一樣，遵循convention以(2,1,0)做
    p = image.transpose(2, 1, 0)
    print('Calculating surface')
    # verts是所有頂點，faces是所有三角形列表
    verts, faces, norm, val = measure.marching_cubes_lewiner(p, threshold, spacing=(1, 1, 1),
                                                             gradient_direction='descent', step_size=step_size,
                                                             allow_degenerate=True)
    return verts, faces

def get_metadata(ds):
    # Extract relevant metadata
    metadata = {
            # 'PatientName': ds.PatientName,
            # 'StudyDescription': ds.StudyDescription,
            # 'Rows': ds.Rows,
            # 'Columns': ds.Columns,
            'PixelSpacing': ds.PixelSpacing,
            'ImageOrientationPatient': ds.ImageOrientationPatient,
            'ImagePositionPatient': ds.ImagePositionPatient,
            'SliceThickness': ds.SliceThickness,
            # Add more fields as needed
        }
    return metadata
