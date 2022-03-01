// The Vue build version to load with the `import` command
// (runtime-only or standalone) has been set in webpack.base.conf with an alias.
import Vue from 'vue'
import App from './App'
import router from './router'
import ROSLIB from 'roslib'
// import * as ros3d from "ros3d";


Vue.config.productionTip = false


// Vue.use(ros3d)
// Vue.use(ROS3D)
/* eslint-disable no-new */

new Vue({
  el: '#app',
  router,
  components: { App },
  template: '<App/>'
})
