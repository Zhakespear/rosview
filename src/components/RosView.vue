<template>
  <div class="overview">
    <div class="navBar">
      <div class="navIcon" @click="openTopicList"></div>
      <div class="title">ROSview</div>
    </div>
    <div>
      <GbDrawer title="ROS Topic List" :display.sync="drawer"
              :width="drawerwidth" @closedrawer="this.drawer=false" @refreshTopic="getTopicLists">
        <div class="topicList" v-for="(value,index) in lists.topics">
          <div class="topicName" @click="initviewer(index)">{{value}}</div>
        </div>
      </GbDrawer>
    </div>
    <div class="showarea" id="showingarea">
      <BasicViewer class="subdiv"
                   :topic="value"
                   :key="value.name"
                   @closeviewer="closeviewer"
                   v-for="(value,index) in onwindows"></BasicViewer>
    </div>



  </div>
</template>

<script>
import GbDrawer from "@/components/Tools/GbDrawer";




import ROSLIB from "roslib";
import * as ROS3D from "ros3d";
import BasicViewer from "@/components/viewer/BasicViewer";
export default {
  name: 'Rosview',
  data () {
    return {
      ros: null,
      connected: false,
      url: 'ws://localhost:9090',
      lists: {
        topics: [],
        types: []
      },
      showingTopics: {},
      onwindows:[],
      drawertitle:"drawertitle",
      drawer:false,
      drawerwidth: '300px'


    }
  },
  methods: {

    initviewer(i) {
      let name = this.lists.topics[i]

      if(this.onwindows.some(ele => ele.name === name) ) return

      let type = this.lists.types[i]
      // let stype = type.split('/').pop()

      this.onwindows.push({name:name,type:type})


    },
    closeviewer(name) {
      this.onwindows.splice(this.onwindows.findIndex(item => item.name === name),1)
      // console.log(this.onwindows);

    },
    openTopicList(){
      this.getTopicLists()
      this.drawer = true
    },




   initROS() {
      this.ros = new ROSLIB.Ros({
        url: this.url
      });
//判断是否连接成功并输出相应的提示消息到web控制台
      this.ros.on('connection', () => {
        this.connected = true;

        console.log('Connected to websocket server.');
      });
      this.ros.on('error', function (error) {
        console.log('Error connecting to websocket server: ', error);
        console.warn('Please check if you have started rosbridge by running command in terminal:' + '\n' + 'roslaunch rosbridge_server rosbridge_websocket.launch')
      });
      this.ros.on('close', function () {
        console.log('Connection to websocket server closed.');
      });
     this.$store.commit('setros',this.ros)
   },

    //publish a topic once
    // publishTopic(topic,topicMessage) {
    //   let type = new ROSLIB.Topic(topic);
    //   let message = new ROSLIB.Message(topicMessage);
    //   type.publish(message);
    // },

    //get all the topics
    getTopicLists() {
      //two callback: success and failure
      //received topic names and types
      this.ros.getTopics((call) => {
          if (this.lists.topics !== call.topics){
            this.lists.topics=call.topics;
            this.lists.types=call.types;
          }
        },
        (call) => {
          console.log('error:failed to load topic lists',call);
        }
      );
    },


  },
  mounted() {
    this.initROS();
    this.getTopicLists();

  },


  components: {
    BasicViewer,
    GbDrawer,

  }
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.overview{
  width: 100%;
  height: 100%;
  background: rgba(0,0,0,.87);
  position: absolute;
  /*margin: 0;*/
  top: 0;
  left: 0;
  /*overflow: auto;*/
  /*overflow-x: scroll;*/
}
.navBar{
  text-align: left;
  /*width: 100%;*/
  height: 60px;
  background: rgb(63,81,181);
  overflow-x: visible;
}
.navIcon{
  cursor: pointer;
  left: 10px;
  display: inline-block;
  width: 48px;
  height: 48px;
  margin: 6px;
  background-repeat: no-repeat;
  background-image: url("../assets/nav.svg");
}

.title{
  font-weight: bold;
  font-size: 20px;
  margin-left: 20px;
  position: absolute;
  display: inline-block;
  color: aliceblue;
  line-height: 60px;
  height: 60px;
}
.topicList {
  text-align: left;
}
.topicName {
  color: #c0c0c0;
  padding: 15px 0 15px 20px;
  text-align: left;
}
.topicName:hover {
  background: #e0e0e0;
  color: #404040;
}

.showarea {
  position: relative;
  display: flex;
  flex-flow: column wrap;
  align-content: flex-start;
  height: calc(100% - 60px);
  overflow-x: auto;

}
.subdiv {
  color: white;
  margin: 10px;
  /*resize: both;*/
  width: calc(50% - 20px);
  height: calc(50% - 20px);
  position: relative;
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
