<template>
  <div id="status">
    <object id="panel" data="vehicle_status.svg" @load="onsvgdone"></object>
  </div>
</template>

<script>
export default {
  name: "StatusViewer",
  data() {
    return {
      svgdone:false,
    }
  },
  methods: {
    // initStatus() {
    //   this.statustopic = new ROSLIB.Topic({
    //     ros : this.$store.state.ros,
    //     name : '/vehicle_status',
    //     messageType : 'vehicle_status_msgs/vehicle_status',
    //   })
    //   this.statustopic.subscribe(this.onstatus )

    // },
    onsvgdone() {
      this.svgdone = true
    },
    onstatus(msg) {
      if(this.svgdone!==true) return
      const panel = document.getElementById('panel').getSVGDocument()
      // steering_wheel_angle
        panel.getElementById('angle').innerHTML = `${Math.abs(msg.steering_wheel_angle).toFixed(1)}°`
      if(msg.steering_wheel_angle>0) {
        panel.getElementById('leftarrow').style.fill = '#0f0'
        panel.getElementById('rightarrow').style.fill = 'none'
      } else if(msg.steering_wheel_angle>0) {
        panel.getElementById('leftarrow').style.fill = 'none'
        panel.getElementById('rightarrow').style.fill = '#0f0'
      } else {
        panel.getElementById('leftarrow').style.fill = 'none'
        panel.getElementById('rightarrow').style.fill = 'none'
      }
      // speed
      // let keyframes = findKeyframesRule(animName);
      const needlesvg = panel.getElementById('needle')
      const angle = msg.vehicle_spd * 3.6 /360 * 270
      needlesvg.style.transformOrigin = "50% 50%"
      needlesvg.style.transition = "transform 1s"
      needlesvg.style.transform = `rotateZ(${angle}deg)`;
      panel.getElementById('spd').innerHTML = `${(msg.vehicle_spd * 3.6).toFixed(1)}km/h`
      // horizon_acc
      // console.log(panel.getElementById('hat'));
      panel.getElementById('hat').setAttribute('transform',`translate(715.2441,${194 + 10 * msg.Longitudinal_acceleration})`)
      panel.getElementById('hac').setAttribute('y1',`${200 + 10 * msg.Longitudinal_acceleration}`)
      panel.getElementById('hac').setAttribute('y2',`${200 + 10 * msg.Longitudinal_acceleration}`)
      // shift
      switch (msg.gear_status) {
        case 0:
          panel.getElementById('shift').innerHTML = 'P'
          break;
        case 1:
          panel.getElementById('shift').innerHTML = 'R'
          break;
        case 2:
          panel.getElementById('shift').innerHTML = 'N'
          break;
        case 3:
          panel.getElementById('shift').innerHTML = 'D'
          break;
        default:
          panel.getElementById('shift').innerHTML = '?'
          break;
      }
    },

  },
  mounted() {
    // let mapdom = document.getElementById("map");
    // mapdom.style.
    // this.initStatus()
  },
  beforeUnmount() {
    // this.statustopic.unsubscribe()
  }
}
</script>

<style scoped>
#status {
  /* height: 600px; */
  height: 100%;

}
#panel {
  position: absolute;
  height: calc(100% - 44px);
  width: 100%;
}

#horizoncur {
  position: absolute;
}

</style>