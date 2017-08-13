import {IMU} from './imu.jsx'

var DynamicSearch = React.createClass({

  // sets initial state
  getInitialState: function(){
    // Wtf is this
    console.log("My domain: " + document.domain);
    console.log("My port: " + location.port);
    

    var my_state = { searchString : '', countries:[]};
    console.log(my_state);
    return my_state
  },

  componentDidMount: function() {
    var socket = io.connect('http://' + document.domain + ':' + location.port);
		socket.on('connect', function() {
		  console.log("i'm connected!");
    });	
    socket.on('message', function(message) {
      console.log("i got mail: " + message);
    });
  
    self = this
    socket.on('countries', function(message) {
      self.setState({countries:message});
      console.log("new countries " + message);
      self.forceUpdate();
    });
    this.state.socket = socket
  },

  // sets state, triggers render method
  handleChange: function(event){
    // grab value form input box
    this.setState({searchString:event.target.value});
    this.state.socket.send(this.state.searchString);
    this.state.socket.emit('test', this.state.searchString);
		console.log("scope updated!");
  },

  render: function() {

    var countries = this.state.countries;
    var searchString = this.state.searchString.trim().toLowerCase();

    // filter countries list by value from input box
    if(searchString.length > 0){
      countries = countries.filter(function(country){
        return country.toLowerCase().match( searchString );
      });
    }

    return (
      <div>
        <input type="text" value={this.state.searchString} onChange={this.handleChange} placeholder="Search!" />
        <ul>
          { countries.map(function(country){ return <li>{country} </li> }) }
        </ul>
      </div>
    )
  }

});

// list of countries, defined with JavaScript object literals
var countries = [
  {"name": "Sweden"}, {"name": "China"}, {"name": "Peru"}, {"name": "Czech Republic"},
  {"name": "Bolivqia"}, {"name": "Latvia"}, {"name": "Samoa"}, {"name": "Armenia"},
  {"name": "Greenland"}, {"name": "Cuba"}, {"name": "Western Sahara"}, {"name": "Ethiopia"},
  {"name": "Malaysia"}, {"name": "Argentina"}, {"name": "Uganda"}, {"name": "Chile"},
  {"name": "Aruba"}, {"name": "Japan"}, {"name": "Trinidad and Tobago"}, {"name": "Italy"},
  {"name": "Cambodia"}, {"name": "Iceland"}, {"name": "Dominican Republic"}, {"name": "Turkey"},
  {"name": "Spain"}, {"name": "Poland"}, {"name": "Haiti"}
];

ReactDOM.render(
  <DynamicSearch items={ countries } />,
  document.getElementById('main')
);
