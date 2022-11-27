<template>
  <div>
    <div id="geobox" ref="geodiv"></div>
<!--    <div @click="renderpath"></div>-->
<!--    <div @click="renderobs"></div>-->
  </div>

</template>

<script>
import ROSLIB from "roslib";

let scene,obs,path
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TDSLoader } from 'three/examples/jsm/loaders/TDSLoader.js'
export default {
  name: "ThreeViewer",
  data() {
    return {
      renderer : null,
      camera : null,
      controls : null,
      datatype : [1,1,2,2,4,4,4,8],
      obstopic : null,
      pathtopic : null,

    }
  },
  methods: {
    init() {

      let height = window.getComputedStyle(this.$refs.geodiv).height
      let width = window.getComputedStyle(this.$refs.geodiv).width
      height = height.substring(0,height.length-2)
      width = width.substring(0,width.length-2)
      scene = new THREE.Scene();
      this.camera = new THREE.PerspectiveCamera( 45, width / height, 0.1, 1000 );
      this.renderer = new THREE.WebGLRenderer({ alpha:false });
      this.renderer.setSize( width, height );
      this.$refs.geodiv.appendChild( this.renderer.domElement );
      this.camera.position.z = 20;
      this.renderer.render(scene,this.camera)
      this.createControls()
      // camera.position.x = 1
      // camera.position.y = 0
      // camera.lookAt(0,0,0)
      this.pathtopic = new ROSLIB.Topic({
        ros : this.$store.state.ros,
        name : '/pub_path',
        messageType : 'path_msgs/path_points',
      })
      this.obstopic = new ROSLIB.Topic({
        ros : this.$store.state.ros,
        name : '/perception/lidar_cells',
        messageType : 'lidar_msgs/cells',
      })


    },
    initpath() {

    },
    initdata() {
      //add points
      let geometry = new THREE.BufferGeometry();
      let points = new Float32Array( 150000 );
      let colors = new Float32Array(200000);
      geometry.setAttribute( 'position', new THREE.BufferAttribute( points,3  ) );
      geometry.setAttribute('color', new THREE.BufferAttribute(colors, 4,false ));
      geometry.setDrawRange( 0, 1 );

      let material = new THREE.MeshBasicMaterial( { vertexColors: THREE.VertexColors, } );
      let mesh = new THREE.Points( geometry, material );
      scene.add(mesh)


    //  add obs
      obs = new THREE.Group()
      scene.add( obs );
      // obs.add(new THREE.Mesh(new THREE.BoxGeometry( 1, 1, 1 ), new THREE.MeshBasicMaterial( {color: 0x00ff00} )))
    //  add path
      path = new THREE.Group()
      scene.add( path );

    },
    addlight() {
      let spotLight1 = new THREE.DirectionalLight(0xFFFFFF,0.3)
      let spotLight2 = new THREE.DirectionalLight(0xFFFFFF,0.3)
      let spotLight3 = new THREE.DirectionalLight(0xFFFFFF,0.3)
      let spotLight4 = new THREE.DirectionalLight(0xFFFFFF,0.3)
      let spotLight5 = new THREE.DirectionalLight(0xFFFFFF,0.15)
      spotLight1.position.set(50,50,50)
      spotLight2.position.set(-50,50,50)
      spotLight3.position.set(50,-50,50)
      spotLight4.position.set(-50,-50,50)
      spotLight5.position.set(0,0,50)
      scene.add(spotLight1)
      scene.add(spotLight2)
      scene.add(spotLight3)
      scene.add(spotLight4)
      scene.add(spotLight5)
    },
    initcar() {
      let loader = new TDSLoader();
      loader.load( 'sCar.3DS', obj => {
        scene.add(obj);
        this.render()
      } ,null , err => console.log(err));
    },
    render() {
      this.renderer.render(scene,this.camera)
    },

    updatepath(flag) {
      if(flag) {
        this.pathtopic.subscribe( this.onpath )
      } else {

        this.pathtopic.unsubscribe()
        for(let i in path.children) {
          if (path.children[i].type === 'Mesh') {
            // console.log(obs.children[i]);
            path.children[i].geometry.dispose();
            path.children[i].material.dispose();
          }
        }
        scene.remove(path)

      }


    },
    onpath(msg) {
      for(let i in path.children) {
        if (path.children[i].type === 'Mesh') {
          // console.log(obs.children[i]);
          path.children[i].geometry.dispose();
          path.children[i].material.dispose();
        }
      }
      scene.remove(path)
      let points = []
      for(let i in msg.Lx){
        points.push( new THREE.Vector3(msg.Lx[i] , msg.Ly[i] , 0))
      }
      let curve = new THREE.CatmullRomCurve3(points)
      curve.getPoints( 200)
      let geometry = new THREE.TubeGeometry( curve, 200, 0.05, 8, false )
      let material = new THREE.LineBasicMaterial( { color: 0xe79c00 } );
      path = new THREE.Mesh( geometry, material )
      scene.add(path)
      this.render()

    },

    updateobs(flag) {
      if(flag) {
        this.obstopic.subscribe( this.onobs )
      } else {
        this.obstopic.unsubscribe()
        for(let i in obs.children) {
          if (obs.children[i].type === 'Mesh') {
            // console.log(obs.children[i]);
            obs.children[i].geometry.dispose();
            obs.children[i].material.dispose();
          }
        }
        scene.remove(obs)
      }



    },
    onobs(msg) {
      for(let i in obs.children) {
        if (obs.children[i].type === 'Mesh') {
          // console.log(obs.children[i]);
          obs.children[i].geometry.dispose();
          obs.children[i].material.dispose();
        }
      }
      scene.remove(obs)
      //
      // if(obs !== undefined) {
      //   obs.traverse( obj => {

      // }
      obs = new THREE.Group()

      let position,scale
      for (let i in msg.cells) {
        position = { x:msg.cells[i].x , y:msg.cells[i].y , z:msg.cells[i].zmean }
        scale = { x:msg.cells[i].x_max-msg.cells[i].x_min , y:msg.cells[i].y_max-msg.cells[i].y_min , z:msg.cells[i].zmax-msg.cells[i].zmin }
        let box = new THREE.Mesh(new THREE.BoxGeometry( scale.x, scale.y, scale.z ), new THREE.MeshBasicMaterial( {color: 0xff0000} ))
        box.position.set(position.x , position.y , position.z)
        obs.add(box)

      }
      scene.add(obs)
      this.render()


    },

    updatepoints(msg) {
      scene.children[0].geometry.attributes.position.needsUpdate = false;
      scene.children[0].geometry.attributes.color.needsUpdate = false;
      let pointdata = msg.data
      let pointview = new DataView(pointdata.buffer);

      let points = scene.children[0].geometry.attributes.position.array
      let colors = scene.children[0].geometry.attributes.color.array



      for(let i=0; ( i * msg.point_step + pointdata.byteOffset )<pointview.byteLength - 10 ; i++) {
        let offset =pointdata.byteOffset + (i * msg.point_step);
        // if (offset>pointview.byteLength) break
        points[3*i] = pointview.getFloat32(offset, true);
        points[3*i+1] = pointview.getFloat32(offset+4, true);
        points[3*i+2] = pointview.getFloat32(offset+8, true);

        let rgb = this.getRgbByPosition(points[3*i+2])

        colors[4*i] = rgb[0]
        colors[4*i+1] = rgb[1]
        colors[4*i+2] = rgb[2]
        colors[4*i+3] = 1

      }

      scene.children[0].geometry.setDrawRange(0,(pointview.byteLength-pointdata.byteOffset-10)/msg.point_step)
      scene.children[0].geometry.computeBoundingBox();
      scene.children[0].geometry.computeBoundingSphere();
      scene.children[0].geometry.attributes.position.needsUpdate = true;
      scene.children[0].geometry.attributes.color.needsUpdate = true;
      this.render()
    },
    getRgbByPosition(v) {
      let vmin = -3
      let vmax = 13
      let c = [255, 255, 255];

      if (v < vmin)
        v = vmin;
      if (v > vmax)
        v = vmax;
      let dv = vmax - vmin;
      if(dv < 1e-2) dv = 1e-2;

      if (v < (vmin + 0.25 * dv)) {
        c[0] = 0;
        c[1] = 4 * (v - vmin) / dv;
      } else if (v < (vmin + 0.5 * dv)) {
        c[0] = 0;
        c[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
      } else if (v < (vmin + 0.75 * dv)) {
        c[0] = 4 * (v - vmin - 0.5 * dv) / dv;
        c[2] = 0;
      } else {
        c[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
        c[2] = 0;
      }

      return c
    },
    createControls() {
      // 创建控件对象
      this.controls = new OrbitControls(this.camera, this.renderer.domElement)

      this.controls.target = new THREE.Vector3(0, 0, 0);//控制焦点

      this.controls.autoRotate = false;//将自动旋转关闭
      // let clock = new THREE.Clock();//用于更新轨道控制器
      // 监听鼠标、键盘事件
      this.controls.addEventListener('change', this.render)

    },
    // animate() {
    //   requestAnimationFrame(this.animate);
    //   this.render()
    // },


  },
  mounted() {
    // console.log(this.$refs.geodiv.getBoundingClientRect());

    this.init()
    this.initcar()
    this.initdata()
    this.addlight()
    // this.animate()


  }
}
</script>

<style scoped>
#geobox {
  height: 500px;
}
canvas {
  width: 100%;
  height: 100%;
  display: inline-block;
}

</style>