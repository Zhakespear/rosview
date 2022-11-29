<template>
  <div id="map"></div>
</template>

<script>
import * as maptalks from 'maptalks'
import 'maptalks/dist/maptalks.css'
export default {
  name: "MapViewer",
  data () {
    return {
      map:null,
      layer:null,
      message:null,
      marker:null,

      // screenWidth: document.body.clientWidth, // 屏幕宽度
		  screenHeight: document.body.clientHeight, // 屏幕高度
    }
  },
  methods: {
    initmap(){
      this.map = new maptalks.Map('map', {
        center: [0,0],
        zoom: 14,
        baseLayer: new maptalks.TileLayer('base', {
          urlTemplate: 'https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png',
          subdomains: ['a','b','c','d'],
          // attribution: '&copy; <a href="http://osm.org">OpenStreetMap</a> contributors, &copy; <a href="https://carto.com/">CARTO</a>'
        })
      });
      this.layer = new maptalks.VectorLayer('vector').addTo(this.map);
      this.marker = new maptalks.Marker(
          // center.sub(0.009, 0),
          [0,0],
          {
            'symbol' : {
              'markerFile'   : require('../../assets/location.svg'),
              'markerWidth'  : 40,
              'markerHeight' : 40,
              'markerDx'     : 0,
              'markerDy'     : 0,
              'markerOpacity': 1
            }
          }
      ).addTo(this.layer);
    },
    updatelocation(msg) {

      this.message = msg

      this.marker.setCoordinates([msg.longitude,msg.latitude])
      // console.log(this.marker.getCoordinates());
    },
    gotocenter(flag) {
      if(flag) {
        this.timer = setInterval(() => {
          this.map.animateTo({
                // zoom: 17,
                center: [this.message.longitude,this.message.latitude]
              }
              , {
                duration: 1
              }
          )
        },100)
      } else {
        clearInterval(this.timer)
      }

    }

  },
  mounted() {
    // let mapdom = document.getElementById("map");
    // mapdom.style.
    this.initmap()
    setTimeout(() => {
      this.map.animateTo({
            // zoom: 17,
            center: [this.message.longitude,this.message.latitude]
          }
          , {
            duration: 1
          }
      )
    },1000)


  },
  beforeUnmount() {
    if(this.timer){
      clearInterval(this.timer)
    }

  }
}
</script>

<style scoped>
#map {
  /* height: 600px; */
  height: 100%;

}

</style>