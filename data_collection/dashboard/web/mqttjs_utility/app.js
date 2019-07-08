

// // Create a client instance
// client = null;
client = null;
connected = false;

// Things to do as soon as the page loads
document.getElementById("clientIdInput").value = 'js-utility-' + makeid();

function makeid()
{
    var text = "";
    var possible = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";

    for( var i=0; i < 5; i++ )
        text += possible.charAt(Math.floor(Math.random() * possible.length));

    return text;
}


function connectionToggle(){

    if(connected){
        disconnect();
    } else {
        connect();
    }
}

function connect()
{

    var hostname = document.getElementById("hostInput").value;
    var port = document.getElementById("portInput").value;
    var clientId = document.getElementById("clientIdInput").value;
    var user = document.getElementById("userInput").value;
    var pass = document.getElementById("passInput").value;


    var options = {
        port: Number(port),
        host: hostname,
        clientId: clientId + Math.random().toString(16).substr(2, 8),
        username: user,
        password: pass,
        keepalive: 60,
        reconnectPeriod: 1000,
        protocolId: 'MQIsdp',
        protocolVersion: 3,
        clean: true,
        encoding: 'utf8'
    };

    client = mqtt.connect('mqtt://10.223.74.110', options) // you add a ws:// url here

    client.on('connect', function() { // When connected

    var statusSpan = document.getElementById("connectionStatus");
    //statusSpan.innerHTML = "Connected to: " + context.invocationContext.host + ':' + context.invocationContext.port + context.invocationContext.path + ' as ' + context.invocationContext.clientId;

    setFormEnabledState(true); 
    });

    connected = true;
    console.log('connected', connected);
    on_message();
    setTimeout(start, 1500);
}

function start()
{

    console.log (" starting...");   
    client.publish("nodes/all", "start");


    // subscribe to a topic
    client.subscribe('nodes/list/all', function() {
    }); 

    // subscribe to a topic
    client.subscribe('nodes/memoryChecker', function() {
    }); 

    // subscribe to a topic
    client.subscribe('nodes/nodedetails', function() {
    }); 
}


    // Sets various form controls to either enabled or disabled
function setFormEnabledState(enabled){

    // Connection Panel Elements
    if(enabled){
        document.getElementById("clientConnectButton").innerHTML = "Disconnect";
    } else {
        document.getElementById("clientConnectButton").innerHTML = "Connect";
    }
    document.getElementById("hostInput").disabled = enabled;
    document.getElementById("portInput").disabled = enabled;
    document.getElementById("clientIdInput").disabled = enabled;
    // document.getElementById("pathInput").disabled = enabled;
    document.getElementById("userInput").disabled = enabled;
    document.getElementById("passInput").disabled = enabled;
}

function subscribe(topic) {
    client.subscribe("mqtt/demo")
}

function disconnect(){
    console.info('Disconnecting from Server');
    client.end();
    var statusSpan = document.getElementById("connectionStatus");
    statusSpan.innerHTML = 'Connection - Disconnected.';
    connected = false;
    setFormEnabledState(false);
}

function createProgressBar2(path) {

    var bar = new ldBar(path.id, {
        "stroke": '#00f',
        "stroke-width": 10,
        "preset": "stripe",
        "value":path.space
    });

    $(path.id).addClass('label-center');
    $(path.id).css({"width":"30%", "height":"10%", "margin":"auto"});
}

function moveBar2(path) {
    console.log("in move bar2");
    var bar = document.getElementById(path.id).ldBar;
    bar.set(path.movement);
}

function on_message() {
    // subscribe to a topic
    client.subscribe('dummy', function() {
    // when a message arrives, do something with it
    client.on('message', function(topic, message, packet) {
    console.log("Received '" + message + "' on '" + topic + "'");
    
    if ( topic == "nodes/list/all")
    {
        var obj = JSON.parse(message);
        console.log('payload size' , obj.nodelist.length);
        var nodelist = "";

        for ( i=0; i < obj.nodelist.length; i++) 
        {
          console.log('node name at ', i , 'is :', obj.nodelist[i]);
          nodelist += obj.nodelist[i] + "</br>";
      }
      document.getElementById('listofnodes').innerHTML = nodelist;
    }
    else if (topic == "nodes/memoryChecker")
    {
        var obj = JSON.parse(message);
        for (i=0; i< obj.paths.length; i++) 
        {
            var available_space =  parseInt(obj.paths[i].available_Space);
            var total_space =  parseInt(obj.paths[i].total_Space);
            var secs_left = " : " + obj.paths[i].seconds_left + "secs";
            console.log('secs left ------------------- ', secs_left);

            var host = obj.paths[i].host + ":";

            var filled = (total_space - available_space);
            var pfilled = Math.ceil ((filled / total_space ) * 100);

            var path =obj.paths[i].path;
            var res = path.replace(/\//g, "");
            if(!document.getElementById(res))
            {
                console.log('new ssd found', path);
                var pathStr = "p"  + res;
                $('<p>').attr('id', pathStr).appendTo('#currentProgress');

                var setpara = document.getElementById(pathStr);
                $(setpara).text(host + path + secs_left);
                $(setpara).css({"text-align":"center", "color":"green", "text-decoration": "underline", " text-transform": "uppercase"});

                var div =  $('<div id="'+res+'" ></div>');
                $("#currentProgress").append(div);

                console.log('before createProgressBar1');
                var proObj = { id: document.getElementById(res), idstr :res, actual:path, space:pfilled};
                createProgressBar2(proObj);  
            }
            else
            {
                console.log('old ssd modifications',pathStr);

                var pathStr = "p" + res;
                var setpara = document.getElementById(pathStr);
                $(setpara).text(host + path + secs_left);
                $(setpara).css({"text-align":"center", "color":"green", "text-decoration": "underline", " text-transform": "uppercase"});

                var moveObj = { id:res, movement:pfilled };

                console.log('calling move bar');
                moveBar2(moveObj);
            }
        }
    }
    else if (topic == "nodes/nodedetails")
    {
        var obj = JSON.parse(message);
        //document.getElementById('nodeDetails').innerHTML = message.payloadString;
        console.log(' obj.nodes.length ', obj.nodes.length);
        // var topic_name ="", topic_type="", freq="";

        for (i=0; i< obj.nodes.length; i++)
        {
            var node_name = obj.nodes[i].node_name;
            var modifiedNodeName;

            if (node_name.includes("/usb_cam"))
            {
                modifiedNodeName = "usb_camera";
            }
            else if(node_name.includes("gps"))
            {
                modifiedNodeName=  "gps";
            }
            else if (node_name.includes("velodyne"))
            {
                modifiedNodeName = "velodyne";
            }
            else if (node_name.includes("imu"))
            {
                modifiedNodeName = "imu";
            }
            else if (node_name.includes("can"))
            {
                modifiedNodeName = "can";
            }

            if(!document.getElementById(obj.nodes[i].node_name))
            {
                console.log('new node');

                var memmessageTable = document.getElementById("nodedetailsMessageTable").getElementsByTagName('tbody')[0];
                var newMessageRow = memmessageTable.insertRow(0);
                newMessageRow.id = obj.nodes[i].node_name;
                newMessageRow.insertCell(0).innerHTML = obj.nodes[i].node_name;
                newMessageRow.insertCell(1).innerHTML = obj.nodes[i].host;
                newMessageRow.insertCell(2).innerHTML = obj.nodes[i].Active;

                var topic_name ="", topic_type="", freq="";
                for (j=0; j< obj.nodes[i].publications.length; j++)
                {
                  topic_name += obj.nodes[i].publications[j].topic_name + "\n" ; ;
                  topic_type += obj.nodes[i].publications[j].topic_type + "\n" ;
                  freq += obj.nodes[i].publications[j].freq + "\n" ;
              }
              newMessageRow.insertCell(3).innerHTML = topic_name;
              newMessageRow.insertCell(4).innerHTML = freq;

              var x = document.createElement("INPUT");
              x.setAttribute("type", "text");
              x.setAttribute("value","0");
              var txt_id = "fps"+obj.nodes[i].node_name;
              x.setAttribute("id", txt_id);
              x.style["width"] = '80px';
              newMessageRow.insertCell(5).appendChild(x);
          }
          else
          {
            var expectedFps;
            var lastMessageRow = document.getElementById(obj.nodes[i].node_name);
            lastMessageRow.id = obj.nodes[i].node_name;
            lastMessageRow.cells[0].innerHTML = obj.nodes[i].node_name;
            lastMessageRow.cells[1].innerHTML = obj.nodes[i].host;
            lastMessageRow.cells[2].innerHTML =  obj.nodes[i].Active;
            var topic_name ="", topic_type="", freq="";
            document.getElementById(obj.nodes[i].node_name).rowSpan = obj.nodes[i].publications.length;
            for (j=0; j< obj.nodes[i].publications.length; j++)
            {
                topic_name += obj.nodes[i].publications[j].topic_name + "</br>" ; ;
                topic_type += obj.nodes[i].publications[j].topic_type + "</br>" ;
                freq += obj.nodes[i].publications[j].freq + "</br>";

                if (obj.nodes[i].publications[j].topic_name.includes("image_raw") || obj.nodes[i].publications[j].topic_name.includes("gps") || obj.nodes[i].publications[j].topic_name.includes("can") || obj.nodes[i].publications[j].topic_name.includes("velodyne"))
                {
                    var txt_id = "fps"+obj.nodes[i].node_name;
                    expectedFps = parseInt(document.getElementById(txt_id).value);
                    console.log(" Actual freq and expected freq", parseInt(obj.nodes[i].publications[j].freq), expectedFps )

                    if (obj.nodes[i].Active =="IN ACTIVE")
                    {
                      console.log(" In active Node ");
                      $(lastMessageRow).css({ 'background-color' : 'red' });
                  }
                  else if ( expectedFps!=0 && (expectedFps > parseInt(obj.nodes[i].publications[j].freq) ) )
                  {
                      console.log("node is not running as expected ");
                      $(lastMessageRow).css({ 'background-color' : 'grey' });
                  }
                  else
                  {
                      console.log("node is running as expected  ");
                      $(lastMessageRow).css({ 'background-color' : 'lightgreen' });

                  }
                }
            }
            lastMessageRow.cells[3].innerHTML = topic_name;
            lastMessageRow.cells[4].innerHTML = freq;
          } 
        }
    }
    });
    });
}

function publish(topic) {
    client.publish("mqtt/demo", "hello world!")
}