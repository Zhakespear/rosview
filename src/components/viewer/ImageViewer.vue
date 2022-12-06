<template>
  <div id="imgbox">
<!--    <iframe src="http://localhost:8081/stream_viewer?topic=/sensors/camera/image_color"></iframe>-->
    <img ref="img">

  </div>

</template>

<script>
export default {
  name: "ImageViewer",
  data() {
    return {
    }
  },
  props: {
    topicname: {
      type: String,
      default: ''
    },
  },
  mounted() {
    this.$refs.img.src = "http://localhost:8081/stream?topic=" + this.topicname
  },
  methods: {

  }
}
</script>

<style scoped>
#imgbox {
  /* margin: auto; */
  height: 100%;
  width: 100%;
  /*height: 500px;*/
}
#imgbox img {
  object-fit: contain;
  height: 100%;
  width: 100%;

}

</style>