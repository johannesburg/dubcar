
export default class IMU extends React.Component {

  constructor(props) {
    super(props);
    // init state
    this.WIDTH = window.innerWidth;
    this.HEIGHT = window.innerHeight;
    this.SPEED = 0.01
    this.scene = new THREE.Scene();
    this.socket = props.socket
    
    this.cube = this.initCube()
    this.initCamera()
    this.initRenderer()
    this.renderFrame()

  }

  componentDidMount() {
    document.getElementById("renderer").appendChild(this.renderer.domElement);
    console.log("this: " + this);
    console.log(this.socket);
    console.log("this.socket.on: " + this.socket.on);
    // TODO yeah wtf is this
    self = this
    this.socket.on('imu', function(roll, pitch, yaw) {
      self.rotateCube(roll, pitch, yaw);
    });
  }

  rotateCube(roll, pitch, yaw) {
    this.cube.rotation.x = roll * (Math.PI/180);
    this.cube.rotation.z = pitch * (Math.PI/180);
    this.cube.rotation.y = yaw * (Math.PI/180);
  }  
  
  initCamera() {
    this.camera = new THREE.PerspectiveCamera(70, this.WIDTH / this.HEIGHT, 1, 10);
    this.camera.position.set(0, 5, 7);
    this.camera.lookAt(this.scene.position);
  }

  initRenderer() {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(this.WIDTH, this.HEIGHT);
  }

  initCube() {
    console.log('initcube')
    var cube = new THREE.Mesh(new THREE.BoxGeometry(4, 4, 4), new THREE.MeshNormalMaterial());
    console.log('exitcube')
    console.log(this.cube)
    this.scene.add(cube);
    return cube;
  }


  renderFrame() {

    requestAnimationFrame(this.renderFrame.bind(this));
    this.renderer.render(this.scene, this.camera);
  }


  render() {
    return (
      <div id="renderer">
      </div>
    )
  }
}

