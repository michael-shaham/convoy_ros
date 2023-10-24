# Dot files

It is recommended to configure some dot files to make coding in a terminal or 
a docker container easier. Having useful dot files can make using `ssh` much 
easier, especially if you know how to use `tmux`. For example, to use Michael's 
dot files:

```bash
cd ~/ && git clone https://github.com/michael-shaham/dot-files.git
cd dot-files
source .bashrc
update-dot-files
```

With this, we can then source the vehicle and correctly set the 
`ROS_DOMAIN_ID` using the command:

```
source-vehicle <vehicle_number>
```

where `vehicle_number` is the number we assign to the vehicle (i.e., if the 
vehicle is the 5th one set up and has IP 192.168.1.104, then replace 
`vehicle_number` with 4).
