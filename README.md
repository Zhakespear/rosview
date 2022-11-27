# rosviewer

## Project setup
```
npm install
```
### Install Rosbridge && web_video_server
```
sudo apt-get install ros-noetic-web-video-server
sudo apt-get install ros-noetic-rosbridge-server
```
### Config video server
#### Make file:
```
roscd web_video_server
mkdir launch
vim web_video.launch
```
#### Insert configuration: (parameters details on https://wiki.ros.org/web_video_server)
````
<launch>
  <node pkg="web_video_server" type="web_video_server" name="web_video_server_1" output="screen">
    <param name="port" type="int" value="8081" />
    <param name="address" type="string" value="127.0.0.1" />
    <param name="server_threads" type="int" value="1" />
    <param name="ros_threads" type="string" value="2" />

    <param name="width" type="int" value="1280" />
    <param name="height" type="int" value="720" />
    <param name="quality" type="int" value="90" />

  </node>
</launch>   
````

### Start Rosbridge && web video server
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

```
roslaunch web_video_server web_video.launch
```


### Node ver == 16.13.0

### Compiles and hot-reloads for development
```
npm run serve
```

### Compiles and minifies for production
```
npm run build
```

### Customize configuration
See [Configuration Reference](https://cli.vuejs.org/config/).
