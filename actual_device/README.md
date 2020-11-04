# catkin_ws


#buoc 1:
- Tạo shell cmd moi,tao file duoi .sh,copy dong lenh duoi:
	- rosclean purge
	- source ./devel/setup.bash
	- sudo ip link set can0 up type can bitrate 125000
	- ip -details link show can0 
	- source ./devel/setup.bash
	- roslaunch -v --screen sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_mls.yaml

#buoc 2:

- Tao shell moi
- kiem tra cong COM port: ls /dev/ttyUSB*
- VD: /dev/ttyUSB0 (defauft)
 	- roslaunch r2serial_driver r2serial_driver yaml:=consept_rb.yaml

- Neu /dev/ttyUSB1
	- roslaunch r2serial_driver r2serial_driver yaml:=consept_rb.yaml port:= /dev/ttyUSB1