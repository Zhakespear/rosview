<template>
  <el-card class="box-card" shadow="always" :body-style="{padding:'0px',height:'100%',display:'flex',flexFlow:'column'}">
    <div class="card-header">
      <div class="titlename">{{topic.name}}</div>
      <span class="buttons">
<!--        path and obs-->
        <span v-if="viewertype==='PointCloud2'"
              :class="{path: !pathfocus,onpath:pathfocus}"
              @click="focusonpath"
              />
        <span v-if="viewertype==='PointCloud2'"
              :class="{obs: !obsfocus,onobs:obsfocus}"
              @click="focusonobs"/>
        <span v-if="viewertype!=='DataRaw'"
              class="switchicon"
              @click="onchangeviewer"/>
        <span v-if="switchviewer==='NavSatFix'"
              :class="{locateicon: !mapfocus,onlocating:mapfocus}"
              @click="focusonmap"/>
        <span class="closeicon"
              @click="this.$emit('closeviewer',this.topic.name)"/>
      </span>

    </div>
    <div class="card-body">
<!--      basic dataraw-->
      <table v-if="switchviewer==='DataRaw'"
             class="table-body">
        <tr v-for="(value,index) in content" class="oneline">
          <td class="left"
              :title="value.name">{{value.name}}</td>
          <td class="right"
              v-html="value.data"
              @click="changestate($event,index)"/>
        </tr>
      </table>
<!--      gps in map-->
      <map-viewer ref="map"
                  v-if="switchviewer==='NavSatFix'"></map-viewer>
<!--      lidar points-->
      <ThreeViewer ref="geo"
                   v-if="switchviewer==='PointCloud2'"></ThreeViewer>
<!--      videos-->
      <ImageViewer ref="img" :topicname="topic.name"
                   v-if="switchviewer==='Image'"></ImageViewer>

      <StatusViewer ref="status" v-if="switchviewer==='vehicle_status'"></StatusViewer>

    </div>

  </el-card>
</template>

<script>
import ROSLIB from "roslib";
import * as ROS3D from "ros3d";

import MapViewer from "@/components/viewer/MapViewer";
import ThreeViewer from "@/components/viewer/ThreeViewer";
import ImageViewer from "@/components/viewer/ImageViewer";
import StatusViewer from "@/components/viewer/StatusViewer"

export default {
  components: {
    MapViewer,
    ThreeViewer,
    ImageViewer,
    StatusViewer,
  },
  name: "BasicViewer",
  props: {
    topic: {
      type: Object,
      default: {name:'Topic Name',
                type:'Topic Type',
      }
    },
  },
  data() {
    return {
      //   focus markers to center
      mapfocus : false,
      obsfocus : false,
      pathfocus : false,
      // type of the viewer
      viewertype:'',
      //0:basicviewer 1:map
      switchviewer:'',
      //obj to subscribe topic
      currenttopic: null,
      //dataraw on display
      content: null,
      //expand or compress content
      compressflag: [],
      //before expand backup
      backup:[],
      

    }
  },
  methods: {
    focusonmap() {
      this.mapfocus = !this.mapfocus
      this.$refs.map.gotocenter(this.mapfocus)
    },
    focusonobs() {
      this.obsfocus = !this.obsfocus
      this.$refs.geo.updateobs(this.obsfocus)
    },
    focusonpath() {
      this.pathfocus = !this.pathfocus
      this.$refs.geo.updatepath(this.pathfocus)
    },
    changestate(e,i) {
      this.compressflag[i] = !this.compressflag[i]
      if(!this.compressflag[i]) {
        this.backup = this.content[i].data
        this.content[i].data = this.content[i].data.replace(/\n/g, "<br>")
            .replace(/ /g, "&nbsp;")
      } else {
        this.content[i].data = this.backup
      }
    },
    onchangeviewer() {
      if(this.switchviewer!=this.viewertype) {
        this.switchviewer = this.viewertype
      } else {
        this.switchviewer = "DataRaw"
      }

      switch (this.viewertype) {
        case "Image": {
          if (this.switchviewer==="DataRaw") this.currenttopic.subscribe(this.ondata)
          else this.currenttopic.unsubscribe()
          break
        }
      }

    },
    init() {
      this.currenttopic = new ROSLIB.Topic({
        ros : this.$store.state.ros,
        name : this.topic.name,
        messageType : this.topic.type,
      })
      switch (this.viewertype) {
        case "NavSatFix":
          this.switchviewer = this.viewertype
          this.currenttopic.subscribe(this.ondata)
          break
        case "PointCloud2":
          this.initlidar()
          this.switchviewer = this.viewertype
          this.currenttopic.subscribe(this.ondata)
          break
        case "Image":
          this.switchviewer = this.viewertype
          // this.currenttopic.subscribe(this.ondata)
          break
        case "vehicle_status":
          this.switchviewer = this.viewertype
          this.currenttopic.subscribe(this.ondata)
          break
        default: {
          this.switchviewer = "DataRaw"
          this.viewertype = "DataRaw"
          this.currenttopic.subscribe(this.ondata)
        }
      }
      // this.currenttopic.subscribe(this.ondata)

      // console.log('type:',type)


    },
    //init pointcloud2
    initlidar(){
      // Setup a client to listen to TFs.
      let tfClient = new ROSLIB.TFClient({
        ros : this.$store.state.ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        // fixedFrame : '/world'
      });

      let cloudClient = new ROS3D.PointCloud2({
        ros: this.$store.state.ros,
        tfClient: tfClient,
        // rootObject: viewer.scene,
        // topic: '/rfans_driver/rfans_points',
        topic: this.topic.name,
        // material: { size: 0.05, color: 0xff00ff }
        material: { size: 0.05, color: 0xff00ff }
      });
    },
    ondata(msg) {
      switch (this.switchviewer) {
        case "NavSatFix":
          this.$refs.map.updatelocation(msg)
          break

        case "PointCloud2":
          this.$refs.geo.updatepoints(msg)
          break
        case "vehicle_status":
          this.$refs.status.onstatus(msg)
          break
        default:
        // case "DataRaw":
          let temp = ''
          let content = []
          if(this.compressflag.length === 0) {
            for (let index in msg) {
              this.compressflag.push(true)
            }
          }
          let i = 0;
          for (let msgKey in msg) {
            if(msg[msgKey].length>20) {
              temp = JSON.stringify(msg[msgKey].slice(0,25), null, '  ')
            } else {
              temp = JSON.stringify(msg[msgKey], null, '  ')
            }
            if (!this.compressflag[i]) {
              temp = temp.replace(/\n/g, "<br>")
                  .replace(/ /g, "&nbsp;")
            }
            i = i + 1
            content.push({name:msgKey,data:temp})
          }
          this.content = content
          content = null
          break

      }
    }
  },
  beforeUnmount() {
    if(this.topic.type!=='sensor_msgs/Image') {
      this.currenttopic.unsubscribe(this.ondata)
    }

  },
  mounted() {
    this.viewertype = this.topic.type.split('/').pop()
    this.init()
  },

}
</script>

<style scoped>
.card-header {
  background: #303030;
  height: 24px;
  padding: 10px 20px;
  display: flex;
  justify-content: space-between;

}
.titlename {
  display: inline-block;
  height: 24px;
  line-height: 24px;
  width: 30%;
}
.buttons {
  display: inline-block;
  right: 0;
  height: 24px;
  /*margin-right: 10px;*/

}
.card-body {
  background: #404040;
  height: 100%;

}
.buttons span {
  display: inline-block;
  cursor: pointer;
  width: 24px;
  height: 24px;
  margin: 0 10px;

}
.obs {
  background: url("../../assets/obs.svg");
}
.onobs {
  background: url("../../assets/onobs.svg");
}
.path {
  background: url("../../assets/path.svg");
}
.onpath {
  background: url("../../assets/onpath.svg");
}
.locateicon {
  background: url("../../assets/tolocate.svg");
}
.onlocating {
  background: url("../../assets/onlocate.svg");
}
.closeicon {
  background: url("../../assets/close.svg");
}
.switchicon {
  background: url("../../assets/switch.svg");
}

.table-body {
  width: 100%;
  min-height: 30pt;
  table-layout: fixed;
  border-collapse: collapse;
  border-spacing: 2px;
  white-space: nowrap;
}

.oneline {
  border-top: 1px solid rgba(0,0,0,.12);
  border-bottom: 1px solid rgba(0,0,0,.12);
  /*padding: 10px 20px 10px 20px;*/
  /*border-bottom: 1px solid white;*/


}
.oneline .left {
  width: 30%;
  padding: 10px 20px;
  overflow: hidden;
  white-space: nowrap;
  text-overflow: ellipsis;
}
.oneline .right {
  overflow: hidden;
  white-space: nowrap;
  text-overflow: ellipsis;
  width: 70%;
  padding: 10px 20px;

  /*display: inline-block;*/
}



.box-card {
  background: #303030;
  /*width: 480px;*/
  border: none;
  padding: 0;

}

</style>