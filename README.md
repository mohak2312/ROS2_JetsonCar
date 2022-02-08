# ROS2_JetsonCar
Control RC car using jetson neno 2GB with ROS2

![WhatsApp Video 2022-02-07 at 4 10 06 PM (2)](https://user-images.githubusercontent.com/18558942/152900571-91fea270-e74a-4a47-9eed-bcd4d8aea172.gif)
![WhatsApp Image 2022-02-07 at 4 07 43 PM](https://user-images.githubusercontent.com/18558942/152900590-44ff3fbd-4322-4aed-a046-7b451a6162d5.jpeg)

**Problem faced**

1. When you flash L4T jetson jetpack img on microsd card, it will by default allocat all the free memroy for img. so basically if we have 128 GB sdcard and we have 128gb free memory then it will allocate 128GB for img. Where img is only approx ~20gb. we are wasting the memory here. Solution: follow the strict instuction mentioned on the nvidia website to install jetson img on jetson neno 2GB, I had to try 2 or 3 times to get it done. If those instruction didn't work then try unchecking the quick format option while formating the sdcard (not sure 100% work or not, it worked for me)
2.  As jetson jetpack uses LT4 (ubantu 18.0.4), we can't use latest ROS2 foxy, which might have more flexiblity and features, SO I have procceded with ROS2 dashing which supports on 18.0
3. faced challenges during ROS2 installation on windows, ROS2 wiki page instruction for installation on windows did't work. Had to follow alternative method mentiond on same page [https://docs.ros.org/en/dashing/Installation/Windows-Install-Binary.html#alternative-ros-2-build-installation-from-aka-ms-ros]
4. Chocolety installation on windows 10:
    * Problem: It was due to the McAfee antivirus software, which was blocking the bypass installtion.
    * Solution: Make sure poweshell is closed, turn off all your antivirus activity [Ex: real-time scanning, firewall] and now open poweshell with admiinistrative mode and run the command from chocolety website for installation

4. Earlier used goto-statment python library in code, which is not an effective way to implement the code, also as i was using python 3.6.9 on jetson, goto-statment python library worked on that but on windows ROS2 installtion with alternative metod (which mentiond above) we will get the python 3.8 by default. Now to support goto-statment python library, we need python 3.6 or less, so have to get rid of the goto-statment python library.
5. I had to use ** pynput** python lib to read keyboard inputs for motion control for jetson car on the JETSON system, as jetson uses ubantu and it doesn't allow the root directroy to read or write and other python library uses root directory to write/read for keyboard inputs.

