import Vue from 'vue'
import Router from 'vue-router'
import Rosview from '@/components/RosView'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      name: 'Rosview',
      component: Rosview
    }
  ]
})
