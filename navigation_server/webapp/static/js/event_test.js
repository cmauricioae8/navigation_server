var toast_element = document.getElementById('toast_element');
var toast_title = document.getElementById('toast_title');
var toast_body = document.getElementById('toast_body');

var notification_toast_element = document.getElementById('notification_toast_element');
var notification_toast_title = document.getElementById('notification_toast_title');
var notification_toast_body = document.getElementById('notification_toast_body');

function send_message(message){
  console.log("Sending Message to server");
  socketio.emit('messages', data=message);
}
// funcion para atrapar mensajes del servidor
socketio.on("messages", (msg) => {
  console.log(msg.msg); // prints the message associated with the message
});
socketio.on("my_response", (msg) => {
  console.log(msg);
});
socketio.on("on_status_change", (msg) =>{
  console.log(msg.data);
  on_status_change_label.innerHTML = JSON.stringify(msg.data, undefined, 2);

  toast_title.innerHTML = "Status change for: " + Object.keys(msg.data)[0];
  var module_data = msg.data[Object.keys(msg.data)[0]];
  var text_body = "";
  Object.keys(module_data).forEach(key => {
    text_body += `<b>${key}</b>: ${JSON.stringify(module_data[key])}<br>`;
  })
  toast_body.innerHTML = text_body;
  
  var toast = new bootstrap.Toast(toast_element);
  toast.show();
})
socketio.on("notifications", (msg) => {
  console.log("Notification:", msg.data);
  notification_toast_title.innerHTML = msg.data.name;
  notification_toast_body.innerHTML = `<b>${msg.data.type}</b>: ${msg.data.message}<br>`;
  var notification_toast = new bootstrap.Toast(notification_toast_element);
  notification_toast.show();
})


/////////////////////////////////////////////////////////////////////////////////////////////

var map_as_event_checkbox = document.getElementById("map_as_event_checkbox");
var with_costmap_checkbox = document.getElementById("with_costmap_checkbox");
var with_scan_checkbox = document.getElementById("with_scan_checkbox");
var with_path_plan_checkbox = document.getElementById("with_path_plan_checkbox");

// ELEMENTS FOR MAPS
var map_image = document.getElementById("map_image");
var costmap_image = document.getElementById("costmap_image");
var button_start_map = document.getElementById("start_map");
var button_stop_map = document.getElementById("stop_map");

var start_mode = document.getElementById("start_mode");

// ELEMENTS FOR CMD_VEL
var button_cmd_forward = document.getElementById("cmd_forward");
var button_cmd_backward = document.getElementById("cmd_backward");
var button_cmd_left = document.getElementById("cmd_left");
var button_cmd_right = document.getElementById("cmd_right");
var button_cmd_stop = document.getElementById("cmd_stop");

var position_element = document.getElementById('position');
var scan_element = document.getElementById('scan');
var path_plan_element = document.getElementById('path_plan');
var on_moving_element = document.getElementById('on_moving');
var on_mode_change_element = document.getElementById('on_mode_change');

// ELEMENTS FOR SET POSE
// var button_pose_set = document.getElementById("pose_set");
// var pose_position_x = document.getElementById("pose_position_x");
// var pose_position_y = document.getElementById("pose_position_y");
// var pose_orientation = document.getElementById("pose_orientation");

// ELEMENTS FOR BATTERY
var button_battery_get = document.getElementById("battery_get");
var button_battery_stop = document.getElementById("battery_stop");
var battery_as_event_checkbox = document.getElementById("battery_as_event_checkbox");
var battery_voltage = document.getElementById("battery_voltage");
var battery_percentage = document.getElementById("battery_percentage");

// ELEMENT FOR on_status_change EVENT
var on_status_change_label = document.getElementById("on_status_change_label");

/////////////////////////////////////////////////////////////////////////////////////////////

// Get maps
$.ajax(
  {
      type: "GET",
      url: document.location.origin+"/maps/maps/",
      success: function(data){
          console.log(data);
          for (var i = 0; i < data.data.length; i++) {
            console.log(data.data[i]);
            document.getElementById('map_select').innerHTML += '<option value="'+data.data[i].id+'">'+data.data[i].name+'</option>';
          }
      }
});

// Get paths
$.ajax(
  {
      type: "GET",
      url: document.location.origin+"/paths/path/",
      success: function(data){

          for (var i = 0; i < data.data.length; i++) {
            console.log(data.data[i]);
            document.getElementById('path_select').innerHTML += '<option value="'+data.data[i].id+'">' + data.data[i].map.name + " - " + data.data[i].name+'</option>';
          }
      }
});

// Set Mode button
start_mode.addEventListener("click", function(){
  console.log("Set mode");
  data = {
    mode: document.getElementById('mode_select').value,
    with_keepout_zones: document.getElementById("with_keepout_zones").checked,
    with_speed_limits: document.getElementById("with_speed_limits").checked,
  }
  if (document.getElementById("map_select").value){
    data.map_id = parseInt(document.getElementById("map_select").value);
  }
  
  $.ajax(
      {
          url: document.location.origin+"/ros/operation_mode/",
          type: "POST",
          //traditional:true,
          headers: {
            'accept': 'application/json',
            'Content-Type': 'application/json'
          },
          data: JSON.stringify(data),
          success: function(data){
              console.log("Set mode success");
          },
          error: function(data){
            console.log(data);
            alert("Error setting mode");
          }
  });
});

// Start Nav button
var start_navigation = document.getElementById("start_navigation");
start_navigation.addEventListener("click", function(){
  console.log("Start navigation");
  data = {
    path_id: document.getElementById('path_select').value,
    mode: document.getElementById('navigation_mode_select').value,
    laps: parseInt(document.getElementById('laps_select').value),
  };
  $.ajax(
      {
        url: document.location.origin+"/navigation/navigation/",
        type: "POST",
        //traditional:true,
        headers: {
          'accept': 'application/json',
          'Content-Type': 'application/json'
        },
        data: JSON.stringify(data),
        success: function(data){
            console.log("Start navigation success");
        }
  });
});


// ACTIONS FOR MAPS
button_start_map.addEventListener("click", function(){
  console.log("get map");
  $.ajax(
      {
          type: "GET",
          url: document.location.origin+"/ros/map/",
          data: {
              // mode: map_as_event_checkbox.checked ? "event" : "unique",
              mode: "event",
              with_costmap: with_costmap_checkbox.checked,
              with_scan: with_scan_checkbox.checked,
              with_path_plan: with_path_plan_checkbox.checked
          },
          success: function(data){
              console.log("Get/Start map success", data);
              map_image.src = data.data.map_data.image;
              if(data.data.costmap_data){
                costmap_image.src = data.data.costmap_data.image;
              }
              if(data.data.scan_data){
                scan_element.innerHTML = `(${data.data.scan_data.length})<br>`;
                for (var i = 0; i < data.data.scan_data.length; i++) {
                  scan_element.innerHTML += `(${data.data.scan_data.points[i].x}, ${data.data.scan_data.points[i].y})<br>`;
                }
              }
              if(data.data.path_plan_data){
                path_plan_element.innerHTML = `(${data.data.path_plan_data.length})<br>`;
                for (var i = 0; i < data.data.path_plan_data.length; i++) {
                  path_plan_element.innerHTML += `(${data.data.path_plan_data.poses[i].position_x}, ${data.data.path_plan_data.poses[i].position_y}, ${data.data.path_plan_data.poses[i].orientation})<br>`;
                }
              }
          }
  });
});
button_stop_map.addEventListener("click", function(){
  console.log("stop map");
  $.ajax(
      {
          type: "GET",
          url: document.location.origin+"/ros/map/",
          data: {
            mode: "stop",
        },
          success: function(data){
              console.log("stop map success");
              console.log(data);
          }
  });
});

// ACTIONS FOR CMD_VEL
button_cmd_forward.addEventListener("click", function(){
  console.log("cmd_forward");
  // max: 0.3 m/s y 0.4 rad/s
  socketio.emit("cmd_vel", data={
    linear_x: 0.4,
    angular_z: 0.0
  });
});
button_cmd_backward.addEventListener("click", function(){
  console.log("cmd_backward");
  // max: 0.3 m/s y 0.4 rad/s
  socketio.emit("cmd_vel", data={
    linear_x: -0.4,
    angular_z: 0.0
  });
});
button_cmd_left.addEventListener("click", function(){
  console.log("cmd_left");
  // max: 0.3 m/s y 0.4 rad/s
  socketio.emit("cmd_vel", data={
    linear_x: 0.0,
    angular_z: 0.4
  });
});
button_cmd_right.addEventListener("click", function(){
  console.log("cmd_right");
  // max: 0.3 m/s y 0.4 rad/s
  socketio.emit("cmd_vel", data={
    linear_x: 0.0,
    angular_z: -0.4
  });
});
button_cmd_stop.addEventListener("click", function(){
  console.log("cmd_stop");
  socketio.emit("cmd_vel", data={
    linear_x: 0.0,
    angular_z: 0.0
  });
});

/////////////////////////////////////////////////////////////////////////////////////////////

// GET ROS EVENTS
// map
socketio.on("map", (msg) => {
  //console.log("map");
  map_image.src = msg.data.image;
});
// costmap
socketio.on("costmap", (msg) => {
  //console.log("costmap");
  costmap_image.src = msg.data.image;
});
// battery
socketio.on("battery", (msg) => {
  console.log("battery");
  battery_voltage.innerHTML = msg.data.voltage.toFixed(2) + " Volts";
  battery_percentage.innerHTML = msg.data.percentage.toFixed(2) + " %";
});
socketio.on('robot_pose', function(data) {
  //console.log(data);
  position_element.innerHTML = `(${data.position_x.toFixed(2)}, ${data.position_y.toFixed(2)}, ${data.orientation.toFixed(2)})`;
});
socketio.on('scan', function(data) {
  //console.log(data);
  // print first 5 points
  var scan_text = `min_dist: ${data.min_distance.toFixed(2)}m to ${data.min_angle}°<br>`;
  var n = 5 < data.length ? 5 : data.length;
  for (var i = 0; i < n; i++) {
    scan_text += `(${data.points[i].x}, ${data.points[i].y})<br>`;
  }
  
  scan_element.innerHTML = `(${data.length})<br>` + scan_text;
});
socketio.on('path_plan', function(data) {
  //console.log('path_plan', data);
  // print first 5 poses
  var path_text = "";
  var n = 5 < data.length ? 5 : data.length;
  for (var i = 0; i < n; i++) {
    path_text += `(${data.poses[i].position_x}, ${data.poses[i].position_y}, ${data.poses[i].orientation})<br>`;
  }
  path_plan_element.innerHTML = `(${data.length})<br>` + path_text;
});
socketio.on('on_moving', function(data) {
  //console.log(data);
  on_moving_element.innerHTML = `on_moving: ${data.on_moving}`;
});
socketio.on('on_mode_change', function(data) {
  //console.log(data);
  on_mode_change_element.innerHTML = data.mode;
});