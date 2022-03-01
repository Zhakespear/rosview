<template>
  <div class="overview">
    <div class="TreeBox" style="width: 19%;  height: 96%;">
      <h1 style="margin: 5%; text-align: center">Topic Configuration</h1>
      <div style="margin-left: 8%;width: 84%">
        <label style="color: white;word-break:break-all" v-for="(value,index) in lists.topics">
          <input type="checkbox" @click="onButton(index, $event)">name: {{ value }}<br>type:{{ lists.types[index] }}<br>
        </label>
      </div>

    </div>
    <div class="ShowBox" id="showarea" style="width: 79%; height: 96%;">
      <viewbox v-for="(value,index) in onwindows" :id="value" :key="index"></viewbox>
<!--      <button @click="lidarloading">load lidar</button>-->
<!--      <div id="viewer"></div>-->
<!--      <Ros3dPointCloud2 topic="/perception/no_ground_points" id="dianyun2"></Ros3dPointCloud2>-->
<!--      <div>-->
<!--        <Ros3dPointCloud2 topic="/perception/no_ground_points" id="dianyun2"></Ros3dPointCloud2>-->

<!--      </div>-->


      <Ros3dViewer :ros="ros" v-if="connected">
<!--        <Ros3dAxes></Ros3dAxes>-->
<!--        <Ros3dGrid></Ros3dGrid>-->
        <Ros3dPointCloud2 topic="/perception/no_ground_points" id="dianyun2"></Ros3dPointCloud2>
      </Ros3dViewer>



<!--      <ros3d-viewer :ros="ros" v-if="connected">-->
<!--        <ros3d-axes></ros3d-axes>-->
<!--        <ros3d-grid></ros3d-grid>-->
<!--        <ros3d-laser-scan topic="/laserscan"></ros3d-laser-scan>-->
<!--      </ros3d-viewer>-->





<!--      <ros3d-viewer :ros="ros" v-if="connected">-->
<!--        <ros3d-axes />-->
<!--        <ros3d-grid />-->

<!--        <ros3d-laser-scan topic="/laserscan"></ros3d-laser-scan>-->
<!--      </ros3d-viewer>-->

    </div>

  </div>
</template>

<script>
// import {Viewer} from "ros3d";
import ROSLIB from "roslib";
import viewbox from "./viewbox";
// import Ros3dViewer from ""
import Ros3dViewer from "vue-ros3djs/src/lib-components/Ros3dViewer"
import Ros3dGrid from "vue-ros3djs/src/lib-components/Ros3dGrid"
import Ros3dAxes from "vue-ros3djs/src/lib-components/Ros3dAxes"
import Ros3dLaserScan from "vue-ros3djs/src/lib-components/Ros3dLaserScan"
import Ros3dPointCloud2 from "vue-ros3djs/src/lib-components/Ros3dPointCloud2"

// import { Ros3dViewer, Ros3dGrid, Ros3dAxes, Ros3dLaserScan } from "vue-ros3djs";
import * as ROS3D from "ros3d";
// import ros3d from "ros3d"
export default {
  name: 'Rosview',
  data () {
    return {
      ros: null,
      connected: false,
      url: 'ws://localhost:9090',
      test: {
        topic: {
          ros : this.ros,
          name : '/cmd_vel',
          messageType : 'geometry_msgs/Twist'
        },
        message: {
          linear : {
            x : 0.1,
            y : 0.2,
            z : 0.3
          },
          angular : {
            x : -0.1,
            y : -0.2,
            z : -0.3
          }
        }
      },
      lists: {
        topics: [],
        types: []
      },
      showingTopics: {},
      onwindows:[],



    }
  },
  methods: {
    // lidarloading(){
    //   // console.log(document.getElementById('viewer'));
    //   // Create the main viewer.
    //   let viewer = new ROS3D.Viewer({
    //     divID : 'viewer',
    //     width : 800,
    //     height : 600,
    //     antialias : true
    //   });
    //
    //   // Setup a client to listen to TFs.
    //   let tfClient = new ROSLIB.TFClient({
    //     ros : this.ros,
    //     angularThres : 0.01,
    //     transThres : 0.01,
    //     rate : 10.0,
    //     fixedFrame : '/world'
    //   });
    //
    //   let cloudClient = new ROS3D.PointCloud2({
    //     ros: this.ros,
    //     tfClient: tfClient,
    //     rootObject: viewer.scene,
    //     topic: 'sensor_msgs/PointCloud2',
    //     // material: { size: 0.05, color: 0xff00ff }
    //     material: { size: 0.05, color: 0xff00ff }
    //   });
    // },
    onButton(index,event){
      let ros=this.ros;
      let name=this.lists.topics[index];
      let type=this.lists.types[index];
      if(event.target.checked==true){
        this.showingTopics[name]=new ROSLIB.Topic({
          ros : ros,
          name : name,
          messageType : type
        });
        this.showingTopics[name].subscribe(this.onTopicMessage);
        this.onwindows.push(name);
        console.log('creatbox');
      }
      else{
        this.showingTopics[name].unsubscribe();
        this.$delete(this.showingTopics,name);
        this.onwindows.splice(this.onwindows.findIndex(item => item == name),1)

        console.log('destroybox');
      }
      // console.log(this.onwindows[0]);

    },
    onTopicMessage(message) {
      console.log("message:",message.data);
    },
    init() {
      this.initROS();
      this.getTopicLists();
    },

   initROS() {
      this.ros = new ROSLIB.Ros({
        url: this.url
      });
      this.test.topic.ros=this.ros;
//判断是否连接成功并输出相应的提示消息到web控制台
      this.ros.on('connection', () => {
        this.connected = true;

        console.log('Connected to websocket server.');
      });
      this.ros.on('error', function (error) {
        console.log('Error connecting to websocket server: ', error);
      });
      this.ros.on('close', function () {
        console.log('Connection to websocket server closed.');
      });

   },

    //publish a topic once
    // publishTopic(topic,topicMessage) {
    //   let type = new ROSLIB.Topic(topic);
    //   let message = new ROSLIB.Message(topicMessage);
    //   type.publish(message);
    // },

    //received topic names and types
    callback(call) {
      console.log("call:",call);
      if (this.lists.topics !== call.topics){
        this.lists.topics=call.topics;
        this.lists.types=call.types;

      }
    },

    //get all the topics
    getTopicLists() {
      this.topiclist = this.ros.getTopics(this.callback,
        function (call) {
          console.log('error:failed to load topic lists',call);
        }
      );
    }

  },

  created() {
    this.init();
    // this.init();


  },
  components: {
    viewbox,
    Ros3dViewer,
    Ros3dGrid,
    Ros3dAxes,
    Ros3dLaserScan,
    Ros3dPointCloud2
  }
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.overview{
  width: 100%;
  height: 100%;
  background: #2c3e50;
  position: absolute;
  margin: 0;
  top:0%;
  left: 0%;
}
.TreeBox{

  /*background: yellow;*/
  border-style: solid;
  border-width: thin;
  border-color: aliceblue;
  position: absolute;
  top: 2%;
  left: 0.5%;

}
.ShowBox{
  border-style: solid;
  border-width: thin;
  border-color: aliceblue;
  position: absolute;
  top: 2%;
  left: 20.5%;

}
h1, h2 {
  color: white;
  margin: 0;
  font-weight: normal;
}
ul {
  list-style-type: none;
  padding: 0;
}
li {
  display: inline-block;
  margin: 0 10px;
}
a {
  color: #42b983;
}
</style>
