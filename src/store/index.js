import { createStore } from 'vuex'

export default createStore({
  state: {
    ros:null
  },
  mutations: {
    setros(state,obj) {
      state.ros = obj
    }
  },
  actions: {
  },
  modules: {
  }
})
