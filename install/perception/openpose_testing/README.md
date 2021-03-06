# BMR6_BACH_SS20_Korecky

**Funktionalität:**

    save_pic:  
        Live-RGB-Bilder der Kamera werden visualisiert  
        Wenn Enter gedrückt wird, werden die aktuellen Bilddaten abgespeichert  
        Dabei kann der gewünschte Ordner, in den gespeichert werden soll, beim Aufrufen als Parameter mitgegeben werden  
        Existiert der Ordner nicht, wird er erstellt  

    publish_pic:  
        Abgespeicherte Bilddaten werden aus einem Ordner extrahiert  
        Diese werden an die Topics, an die die Kamera sie normalerweise published, gepublished  
        Der OpenPose-Wrapper wird ausgeführt und visualisiert und published die Ergebnisse der Auswertung der abgespeicherten Bilder  
        Die Position ausgewählter Körperteile im Kamerakoordinatensystem wird im Terminal ausgegeben  

**Start-Up:**  

    save_pic:  
        "roslaunch save_pic.launch" => Aufruf ohne Parameter, es wird per default ein Ordner namens "testfolder" erstellt und die Daten darin gespeichert  
        "roslaunch save_pic.launch folder:="myFolder" " => Folder namens "myFolder" wird im Ordner "bagfiles" erstellt  
        Wenn Enter gedrückt wird, werden die aktuellen Bilddaten gespeichert (entsprechendes Terminal muss ausgewählt sein)  
        Für jedes gespeicherte Bild wird ein Ordner mit dem Namen "1", "2", "3"... erstellt und die Daten darin gespeichert  

    publish_pic:  
        "roslaunch publish_pic.launch" => Aufruf ohne Parameter, die Files aus dem Ordner "testfolder/1" werden verwendet  
        "roslaunch save_pic.launch folderpath:="myFolder/1" " => Die Files aus "myFolder/1" werden verwendet  

**Subscribed Topics:** 

    get_images.cpp (Teil von save_pic):  
        /camera/color/image_raw  
        /camera/aligned_depth_to_color/image_raw  
        /camera/color/camera_info  
        /camera/depth/color/points  
        /tf_static  
        /EnterPressed  

    getOpenPoseData.cpp (Teil von publish_pic):  
        /frame  

**Published Topics:**

    get_images.cpp (Teil von save_pic):  
        /image_pub  

    take_picture.cpp (Teil von save_pic):  
        /EnterPressed  

    publish_picture.cpp (Teil von publish_pic):  
        /camera/color/image_raw  
        /camera/aligned_depth_to_color/image_raw  
        /camera/color/camera_info  
        /camera/depth/color/points  
        /tf_static  


**Installationsanleitung:**

    -Kamera SDK: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md  
    -ROS-Wrapper für die Kamera: https://github.com/IntelRealSense/realsense-ros  
    -OpenPose: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md#installation  
    -OpenPose-Wrapper: https://github.com/ravijo/ros_openpose  
    -Ncurses-Library: https://www.cyberciti.biz/faq/linux-install-ncurses-library-headers-on-debian-ubuntu-centos-fedora/  
    -Folder "openpose_testing" in Catkin Workspace kopieren  



    