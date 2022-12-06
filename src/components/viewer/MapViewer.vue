<template>
  <div id="map"></div>
</template>

<script>
import * as maptalks from 'maptalks'
import 'maptalks/dist/maptalks.css'
import ROSLIB from "roslib";

export default {
  name: "MapViewer",
  data() {
    return {
      map: null,
      layer: null,
      message: null,
      marker: null,
      gpspath: null,
      tracetopic:null,
    }
  },
  methods: {
    initmap() {
      this.map = new maptalks.Map('map', {
        center: [0, 0],
        zoom: 17,
        baseLayer: new maptalks.TileLayer('base', {
          // urlTemplate: 'https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png',
          urlTemplate: 'roadmap/{z}/{x}/{y}.png',
          subdomains: ['a', 'b', 'c', 'd'],
          // subdomains:['a','b','c']
          // attribution: '&copy; <a href="http://osm.org">OpenStreetMap</a> contributors, &copy; <a href="https://carto.com/">CARTO</a>'
        })
      });
      this.layer = new maptalks.VectorLayer('vector').addTo(this.map);
      this.marker = new maptalks.Marker(
        // center.sub(0.009, 0),
        [0, 0],
        {
          'symbol': {
            'markerFile': require('../../assets/location.svg'),
            'markerWidth': 40,
            'markerHeight': 40,
            'markerDx': 0,
            'markerDy': 0,
            'markerOpacity': 1
          }
        }
      ).addTo(this.layer);
    },
    inittrace() {
      this.tracetopic = new ROSLIB.Topic({
        ros : this.$store.state.ros,
        name : '/global_traces',
        messageType : 'trace_msgs/points',
      })
      this.tracetopic.subscribe(this.ontrace )

      this.gpspath = new maptalks.MultiLineString([[0,0],[100,100]],{
        arrowStyle : null, // arrow-style : now we only have classic
        arrowPlacement : 'vertex-last', // arrow's placement: vertex-first, vertex-last, vertex-firstlast, point
        visible : true,
        editable : true,
        cursor : null,
        draggable : false,
        dragShadow : false, // display a shadow during dragging
        drawOnAxis : null,  // force dragging stick on a axis, can be: x, y
        symbol: {
          'lineColor' : '#ff0000',
          'lineWidth' : 1
        }
      })
      new maptalks.VectorLayer('path', this.gpspath).addTo(this.map);
    },
    updatetrace(flag) {
      if(flag) {
        this.tracetopic.subscribe( this.ontrace )
      } else {
        this.tracetopic.unsubscribe()
      }
    },
    ontrace(msg) {
      // for (const item in msg.points) {
      //   // console.log(item);
      // }
      // console.log(msg);
      let pathpoints = []
      // pathpoints.push([[msg.points[1],msg.points[0]],[msg.points[3],msg.points[2]]])
      for(let i=2;i<msg.points.length;i+=2) {
        // let lon = msg.points[i]
        // let lat = msg.points[i+1]
        // console.log([[msg.points[i-1],msg.points[i-2]],[msg.points[i+1],msg.points[i]]]);
        pathpoints.push([[msg.points[i-1],msg.points[i-2]],[msg.points[i+1],msg.points[i]]])
      }
      // console.log(pathpoints);
      this.gpspath.setCoordinates(pathpoints)
      // console.log(this.gpspath.getCoordinates());

      // console.log(this.gpspath);

    },
    updatelocation(msg) {

      this.message = msg

      this.marker.setCoordinates([msg.longitude, msg.latitude])
      // console.log(this.marker.getCoordinates());
    },
    gotocenter(flag) {
      if (flag) {
        this.timer = setInterval(() => {
          this.map.animateTo({
            // zoom: 17,
            center: [this.message.longitude, this.message.latitude]
          }
            , {
              duration: 1
            }
          )
        }, 500)
      } else {
        clearInterval(this.timer)
      }

    }

  },
  mounted() {
    // let mapdom = document.getElementById("map");
    // mapdom.style.
    this.initmap()

    this.inittrace()
    setTimeout(() => {
      this.map.animateTo({
        // zoom: 17,
        center: [this.message.longitude, this.message.latitude]
      }
        , {
          duration: 1
        }
      )
    }, 1000)


  },
  beforeUnmount() {
    if (this.timer) {
      clearInterval(this.timer)
    }
    this.tracetopic.unsubscribe()


  }
}
</script>

<style scoped>
#map {
  /* height: 600px; */
  height: 100%;

}
</style>