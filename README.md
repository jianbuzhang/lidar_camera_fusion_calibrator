## Requirements

- VTK7.1
- PCL 1.8
- OpenCV 3.4.11


## How to use

### 1. Make data folder

- Format

```
folder_name
├──xxx.png
├──xxx.pcd
├──yyy.png
├──yyy.pcd
├──...
```

You should specify thie image and point cloud data from the same frame for xxx.png and xxx.pcd.

PNG images are only supported.

### 2. Build this project

In this project,

```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

### 3. Run

In build folder,

```
$ ./CalibratorTool
$ ./CalibratorTool <initparam.yaml>
```

### 4. After project run
- Data folder will be create.
- Folder structure
    -data
        -internal_param
        -LeftBackWideAngle
        -LeftFrontWideAngle
        -LeftLongFocus
        -RightBackWideAngle
        -RightFrontWideAngle
        -RightLongFocus
    -log
