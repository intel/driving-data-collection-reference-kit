/*
  Copyright (C)  2018Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 
*/

// ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/topic_manager.h>
#include "ros/network.h"
#include "ros/xmlrpc_manager.h"
#include "XmlRpc.h"

#include <thread>
#include <map>
#include <vector>
#include <chrono>
#include <cmath>
#include <deque>
#include <set>

#include <mosquittopp.h>
#include <json-c/json.h>


// C specified headers
#include <sys/types.h>
#include <sys/statvfs.h>
#include <signal.h>


#define MAX_DEVICES_PER_NODE 8
#define MAX_SSDS_CONNECTED 2
#define MAX_MEM_LEVEL_TO_WARN 10000000000 // 10 GB
#define GIGA_BYTES 1000000000

std::vector<std::string> node_names;          // Vector of node names running
std::map<std::string, std::string> topic_map; // Map to store all the topic names with its data type
std::vector <ros::Subscriber> sub;
std::vector <ros::WallTimer> wtimer;

char hostbuffer[256];

void updateTopicList(std::map<std::string, std::string> &v);

class DummyMsg 
{
public:
  static std::string md5 ;
  static std::string data_type;
  static std::string const &__s_getMD5Sum() { return md5; }
  static std::string const &__s_getDataType() { return data_type; }
  void deserialize(void *) {}
};

std::string DummyMsg::md5 = "*";
std::string DummyMsg::data_type = "/";

class rostopicHZ
{

public:
  double K = 0.0;
  double sum = 0.0;
  double sq_dev = 0.0;
// For min / max tracking.
  std::multiset<double> data;
  double avg_delay;
  double variance;
  std::string hz;
  unsigned int window_size = 10000;
  unsigned int message_count = 0;
  std::deque<double> times;
  std::chrono::system_clock::time_point last;
  unsigned int status_count = 0;

  void status_callback(const ros::WallTimerEvent &event);
  void msg_callback(const DummyMsg &data);


private:
  void add_data_point(double x) {
    if (!data.size()) {
      K = x;
    }

    data.insert(x);
    sum += x - K;
    sq_dev += (x - K) * (x - K);
  }

  void remove_data_point(double x) {
// We only want to remove one element.
    data.erase(data.find(x));
    sum -= (x - K);
    sq_dev -= (x - K) * (x - K);
  }

  double get_min() {
    return *data.begin();
  }

  double get_max() {
    return *std::prev(data.end());
  }

  double get_mean() {
    return K + sum / data.size();
  }

  double get_variance() {
    return (sq_dev - (sum * sum) / data.size()) / data.size();
  }

  unsigned int get_n() {
    return data.size();
  }
};

void rostopicHZ::msg_callback(const DummyMsg &data) {
  auto time = std::chrono::system_clock::now();
  double diff = std::chrono::duration<double>(time - last).count();
  last = time;

  message_count++;

  if (message_count < 2) {
    return;
  }

  times.push_back(diff);
  add_data_point(diff);

  if (get_n() + 1 > window_size) {
    remove_data_point(times.front());
    times.pop_front();
  }
}

void rostopicHZ::status_callback(const ros::WallTimerEvent &event) {
  ROS_DEBUG(" in status callback .........................\n\n\n\n");
  if (status_count == message_count || times.size() < 2) {
    ROS_DEBUG(" No new messages." );
    hz = "No new messages";
    return;
  }

  status_count = message_count;

  double avg_delay = get_mean();
  double variance = get_variance();

  ROS_DEBUG( "Average delay: %lf ",  avg_delay );
  ROS_DEBUG("rate of Hz : %lf ", (double) 1.0 / avg_delay);
  double freq = 1.0 /avg_delay;
  hz = std::to_string(freq);
  ROS_DEBUG("\tmin: %lf s max : %lf s std dev : %lf s window: : %d ", get_min() , get_max(), std::sqrt(variance),get_n() + 1 );
}

class rosTopic
{
public: 
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  rosTopic(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  {
    nh_ = nh;
    priv_nh_ = priv_nh;
  }
  std::string topicName;
  std::string dataType;
  rostopicHZ hz;
  void subscribeToTopics(std::string topic_name, rostopicHZ &topic);

};

class nodes
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  std::string uri;
  std::string host;
  uint32_t port;
  bool alive;
  
public:
  nodes(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  {
    nh_ = nh;
    priv_nh_ = priv_nh;
  }

  std::string node_name;
  std::map<std::string, rosTopic *> topicInfoMap;
  std::map<std::string, rosTopic *> subscribedTopics;
  std::map<std::string, rosTopic *> publishedTopics;
  void lookUpnode();
  void getSubscriptions();
  void getPublications();
  std::string getHost() { return host; }
  uint32_t getPort() { return port; }
  std::string getUri() { return uri; }
  bool isNodeActive() { return alive; }
  void setNodeStatus(bool status) { alive = status; }

};

class memory_status
{
public:
  std::string seconds_left_;
  long long int memory_available_;
  long long int total_size;
  memory_status()
  {
    
  }

  inline std::string getStoragePath()
  {
    return storage_path_;
  }

  inline void setStoragePath(std::string path)
  {
    storage_path_ = path;
  }

  inline long long int getMemory_available()
  {
    return memory_available_;
  }

private:

  std::string storage_path_;

  enum memory_representation
  {
    Bytes,
    KB = 1000,
    MB = 1000000,
    GB = 1000000000,
  };
};

class mqttClient : public mosqpp::mosquittopp
{
public:
  bool start = false;

  mqttClient(const char *client_id, const char *host, int port) : mosquittopp(client_id)
  {
    int keepalive = 60;
    username_pw_set("ad", "ad123");
    connect(host, port, keepalive);
  }

  void on_connect(int connectStatus)
  {
    if (connectStatus != 0)
    {
      ROS_ERROR("Connection Failed ");
      return;
    }
    else
    {
      ROS_INFO(" mosuitto MQTT Connection sucsessfull");
      subscribe(NULL, "nodes/all");
    }
  }

  void on_subscribe(int mid, int qos_count, const int *granted_qos)
  {
    ROS_INFO("subscribe sucsessfull");
  }

  void on_message(const struct mosquitto_message *message)
  {
    ROS_INFO("Recieved Message on Topic %s ", message->topic);
    if(!strcmp(message->topic, "nodes/all"))
    {
      char buffer[51];
      memcpy(buffer, message->payload, message->payloadlen);
      if (!strcmp(buffer, "start"))
      {
        start = true;
      }
      else if (!strcmp(buffer, "stop"))
      {
      }
    }
  }

  void waitForMqttStart()
  {
    while (start == false)
    {
      sleep(3);
    }
  }

  void send_node_list(std::vector<std::string> nodes)
  {
    // { "nodelist": [ "\/dashboard", "\/rosout", "\/usb_cam" ] } //
    if (start)
    {
      json_object *jobj  = json_object_new_object();

      /*Creating a json array*/
      json_object *jarray = json_object_new_array();

      for (auto it=nodes.begin(); it!=nodes.end(); ++it)
      {
        std::string node_name = *it;
        const char * nname = node_name.c_str();
        json_object *jstring1 = json_object_new_string(nname);
        json_object_array_add(jarray,jstring1);
      }
      json_object_object_add(jobj,"nodelist", jarray);

      ROS_DEBUG("The json object created: %sn",json_object_to_json_string(jobj));
      const char *value = json_object_to_json_string(jobj);
      publish(NULL, "nodes/list/all", strlen(value), value, 0, false);
    }
  }

  /* {
{
      "paths": [{
                      "path": "/home/",
                      "avail": "10gb",
                      "ttl": "10sec"
              },
              {
                      "path": "/ad/",
                      "avail": "10gb",
                      "ttl": "10sec"
              }
      ]
}
}*/
  void send_memory_status(memory_status memStatus[MAX_SSDS_CONNECTED])
  {
    json_object *jobj  = json_object_new_object();
    json_object *jarray = json_object_new_array();

    for (int i=0; i < MAX_SSDS_CONNECTED; i++)
    {
      std::string path = memStatus[i].getStoragePath();
      std::string seconds_left = memStatus[i].seconds_left_;
      // long long int memory_available_ = memStatus[i].getMemory_available();
      // long double mAvailable = memory_available_ / GIGA_BYTES;
      long long int memory_available_ = (memStatus[i].getMemory_available()) / GIGA_BYTES;
      long long int tot_mem_ = ceil ((memStatus[i].total_size) / GIGA_BYTES);


      std::string mem_in_gb = std::to_string(memory_available_);
      std::string tot_mem_in_gb = std::to_string(tot_mem_);

      json_object *jobj1  = json_object_new_object();

      json_object_object_add(jobj1, "host", json_object_new_string(hostbuffer));
      json_object_object_add(jobj1, "total_Space", json_object_new_string(tot_mem_in_gb.c_str()));
      json_object_object_add(jobj1, "available_Space", json_object_new_string(mem_in_gb.c_str()));
      json_object_object_add(jobj1, "seconds_left", json_object_new_string(seconds_left.c_str()));
      json_object_object_add(jobj1, "path", json_object_new_string(path.c_str()));
      json_object_array_add(jarray, jobj1);
    }
    json_object_object_add(jobj, "paths", jarray);
    ROS_INFO(" Memory status : %s \n", json_object_to_json_string(jobj));
    const char *value = json_object_to_json_string(jobj);
    publish(NULL, "nodes/memoryChecker", strlen(value), value, 0, false);
  }

  void send_node_status(std::map<std::string, nodes *> map)
  {

    if (start)
    {
      json_object *jobj_nodes = json_object_new_object();
      json_object *jnode_array = json_object_new_array();

      for (auto it=map.begin(); it!=map.end(); ++it)
      {
        nodes *newNode = it->second;
        std::string node_name = it->first;
        std::string node_active =  newNode->isNodeActive() == 1 ? "ACTIVE" : "IN ACTIVE";
        std::string host = newNode->getHost();

        json_object *jobj_node = json_object_new_object();
        json_object *jpub_key = json_object_new_object();
        json_object *jarray_pub = json_object_new_array();

        json_object_object_add(jobj_node, "node_name", json_object_new_string(node_name.c_str()));
        json_object_object_add(jobj_node, "Active", json_object_new_string(node_active.c_str()));
        json_object_object_add(jobj_node, "host", json_object_new_string(host.c_str()));


        for (auto topicinfoIt = newNode->publishedTopics.begin(); topicinfoIt != newNode->publishedTopics.end(); topicinfoIt++)
        {
          rosTopic *topic = topicinfoIt->second;
          ROS_INFO(" \t * %s[%s][%s]", topicinfoIt->first.c_str(), topic->dataType.c_str(), topic->hz.hz.c_str());

          json_object *jobj_pub = json_object_new_object();
          json_object_object_add(jobj_pub, "topic_name", json_object_new_string(topicinfoIt->first.c_str()));
          json_object_object_add(jobj_pub, "topic_type", json_object_new_string(topic->dataType.c_str()));
          json_object_object_add(jobj_pub, "freq", json_object_new_string(topic->hz.hz.c_str()));
          json_object_array_add(jarray_pub, jobj_pub);
        }
        json_object_object_add(jobj_node, "publications", jarray_pub);
        json_object_array_add(jnode_array, jobj_node);
      }
      json_object_object_add(jobj_nodes, "nodes", jnode_array);
      ROS_DEBUG(" node details : %s \n", json_object_to_json_string(jobj_nodes));
      const char *value = json_object_to_json_string(jobj_nodes);
      publish(NULL, "nodes/nodedetails", strlen(value), value, 0, false);
    }
  }

};

class dashboard
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  double cmd_frequecy_ = 100;
  ros::Timer cmd_xmit_cb_;
  int no_of_nodes_running;
std::map<std::string, nodes *> nodemap; // a map for node name to an node.

public:
  memory_status memStatus[MAX_SSDS_CONNECTED];
  std::string path1, path2;
  class mqttClient *mqtt_client;

  dashboard(ros::NodeHandle nh, ros::NodeHandle priv_nh) :mqtt_client(NULL)
  {
   if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
    ros::console::notifyLoggerLevelsChanged();

  ROS_DEBUG(" In constructor \n");
  nh_ = nh;
  priv_nh_ = priv_nh;

  priv_nh_.param("path1", path1, std::string("/dev/sda"));
  priv_nh_.param("path2", path2, std::string("/dev/sdab"));

  std::vector<std::string> nodelist;
  no_of_nodes_running = getNoOfNodesRunning(nodelist);
  if (no_of_nodes_running)
  {
    for (auto i = nodelist.begin(); i!= nodelist.end(); i++)
    {
      std::string node_name = *i;
      if (node_name != "/rosout")
      {
        nodes *new_node= new nodes(nh_, priv_nh_);

        add_node(node_name, new_node);
        new_node->lookUpnode();
        if (std::strcmp(new_node->getHost().c_str(),hostbuffer) != 0)
        {
          delete new_node;
          continue;
        }
        new_node->getSubscriptions();
        new_node->getPublications();
        updateTopicList(topic_map);
        node_names.push_back(node_name);
        nodemap[new_node->node_name] = new_node;
      }
    }
    printDetails(nodemap);
  }
}

void createTimer();
void timerCb(const ros::TimerEvent&);
inline int getNoOfNodesRunning(std::vector<std::string> &);
void add_node(const std::string, nodes *);
void deleteNode();
void printDetails(std::map<std::string, nodes*> map);

};

std::map<std::string, nodes> nmap;

void updateTopicList(std::map<std::string, std::string> &v)
{
// query for the topics
  ROS_DEBUG(" Topics being Published \n");
  std::vector<ros::master::TopicInfo> topicList;
  if (ros::master::getTopics(topicList))
  {
    for (auto i = topicList.begin(); i!= topicList.end(); i++)
    {
     ros::master::TopicInfo topicclass = *i;
     ROS_DEBUG(" TOPIC : %s Data type : %s ", topicclass.name.c_str(), topicclass.datatype.c_str());
     v[topicclass.name] = topicclass.datatype;
   }
 }
}

void dashboard::printDetails(std::map<std::string, nodes *> map)
{
  ROS_INFO(" Nodes Active [%ld] ", node_names.size() );
  for (auto it=map.begin(); it!=map.end(); ++it)
  {
    nodes *newNode = it->second;
    ROS_INFO(" Node   [%s][%s]  ", it->first.c_str(), newNode->isNodeActive() == 1 ? "ACTIVE" : "IN ACTIVE" ) ;

    if (newNode->isNodeActive())
    {
      
      ROS_INFO(" Publications");
      for (auto topicinfoIt = newNode->publishedTopics.begin(); topicinfoIt != newNode->publishedTopics.end(); topicinfoIt++)
      {
        rosTopic *topic = topicinfoIt->second;
        ROS_INFO(" \t * %s[%s][%s]", topicinfoIt->first.c_str(), topic->dataType.c_str(), topic->hz.hz.c_str());
      }

      ROS_INFO(" Subscriptions");
      for (auto topicinfoIt = newNode->subscribedTopics.begin(); topicinfoIt != newNode->subscribedTopics.end(); topicinfoIt++)
      {
        rosTopic *topic = topicinfoIt->second;
        ROS_INFO(" \t * %s[%s][%s]", topicinfoIt->first.c_str(), topic->dataType.c_str(), topic->hz.hz.c_str());
      }

      ROS_INFO(" Uri :");
      ROS_INFO(" \t * %s ", newNode->getUri().c_str());
    }
    ROS_INFO("\n");
  }
  ROS_INFO("\n\n");
}

int dashboard::getNoOfNodesRunning(std::vector<std::string> &nodelist)
{
  if(ros::master::getNodes(nodelist))
  {
    return nodelist.size();
  }
  else return 0;  
}

void dashboard::add_node(const std::string node_name, nodes *n)
{
  n->node_name = node_name;
  n->setNodeStatus(true);
}

void dashboard::deleteNode()
{


}

void dashboard::createTimer()
{
  double timer_duration = 1;
  cmd_xmit_cb_ = nh_.createTimer(ros::Duration(timer_duration), &dashboard::timerCb, this);
}

void dashboard::timerCb(const ros::TimerEvent&)
{
  std::vector<std::string> nodelist;
  int nodesrunning = getNoOfNodesRunning(nodelist);

  if (no_of_nodes_running < nodesrunning)
  {
    no_of_nodes_running = nodesrunning;
    ROS_INFO(" %d New nodes Found \n", nodesrunning - no_of_nodes_running);
    for (auto i = nodelist.begin(); i!= nodelist.end(); i++)
    {
      std::string node_name = *i;
      if (node_name != "/rosout")
      {
        if (node_names.end() == (std::find(node_names.begin(), node_names.end(), node_name)))
        {
          nodes *new_node= new nodes(nh_, priv_nh_);
          add_node(node_name, new_node);
          updateTopicList(topic_map);
          new_node->lookUpnode();
          if (std::strcmp(new_node->getHost().c_str(),hostbuffer) != 0)
          {
            delete new_node;
            continue;
          }
          new_node->getSubscriptions();
          new_node->getPublications();
          node_names.push_back(node_name);
          nodemap[new_node->node_name] = new_node;    
        }
      }
    }
  }
  else if (no_of_nodes_running > nodesrunning)
  {
    no_of_nodes_running = nodesrunning;
    ROS_INFO("%d Nodes Went off .. \n", no_of_nodes_running - nodesrunning);
    for (auto i = nodelist.begin(); i!= nodelist.end(); i++)
    {
      std::string node_name = *i;
      for (auto i = node_names.begin(); i!= node_names.end(); )
      {
        std::string node = *i;
        if (nodelist.end() == (std::find(nodelist.begin(), nodelist.end(), node)))
        {
          i = node_names.erase(i);
          nodemap[node]->setNodeStatus(false);
        }
        else
        {
          ++i;
        }
      }
    }
  }
  printDetails(nodemap);
  if (this->mqtt_client)
  {
    this->mqtt_client->send_node_list(node_names);
    this->mqtt_client->send_node_status(nodemap);  
  }
  
}

void nodes::lookUpnode()
{
  std::string peer_host;
  uint32_t peer_port;

  std::string node = this->node_name;
  XmlRpc::XmlRpcValue request;
  request[0] = ros::this_node::getName();
  request[1] = node;
  XmlRpc::XmlRpcValue resp;
  XmlRpc::XmlRpcValue payload;
  bool success = ros::master::execute("lookupNode", request,
    resp, payload, false); 

  if (!success)
    ROS_ERROR("lookupNode failed ");

  if (static_cast<int>(resp[0]) == 1)
  {
    this->uri = static_cast<std::string>(resp[2]);
    if (!ros::network::splitURI(static_cast<std::string>(resp[2]), peer_host, peer_port))
    {
      ROS_ERROR("Bad xml-rpc URI trying to inspect node at: [%s]", static_cast<std::string>(resp[2]).c_str());
    }
    else
    {
      ROS_DEBUG(" peer host : %s peer port : %d ", peer_host.c_str(), peer_port);
      this->host = peer_host;
      this->port = peer_port;
    }
  }
  else
  {
    ROS_ERROR(" look up error code : %s \n", static_cast<std::string>(resp[2]).c_str());
  }
}

void nodes::getSubscriptions()
{
/* getSubscriptions(caller_id): Retrieve a list of topics that this node subscribes to
   Parameters:
      caller_id (str)
          ROS caller ID. 
    Returns (int, str, [ [str, str] ])
      (code, statusMessage, topicList)
                            topicList -> [ [topic1, topicType1]...[topicN, topicTypeN] ]
*/

 std::string peer_host = this->getHost();
 uint32_t peer_port = this->getPort();
 XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
 XmlRpc::XmlRpcValue req;
 XmlRpc::XmlRpcValue resp;
 req[0] = this->node_name;
 c.execute("getSubscriptions", req, resp);
 
 if (!c.isFault() && resp.valid() && resp.size() > 0 && static_cast<int>(resp[0]) == 1)
 {
   for(int i = 0; i < resp[2].size(); i++)
   {
    std::string topic_name = static_cast<std::string>(resp[2][i][0]);
    std::string dataType =  static_cast<std::string>(resp[2][i][1]);

    auto dtmap = subscribedTopics.find(topic_name);
    if (dtmap == subscribedTopics.end())
    {
      rosTopic *topic = new rosTopic(nh_, priv_nh_);
      topic->topicName = topic_name;
      topic->dataType = dataType;
      subscribedTopics[topic_name] = topic;            
    } 
    ROS_INFO(" topic: %s  topictype: %s ", static_cast<std::string>(resp[2][i][0]).c_str(),  static_cast<std::string>(resp[2][i][1]).c_str());
  }
}
else
{
 ROS_ERROR("Node at: [%s] failed to return subscriptions.", static_cast<std::string>(resp[1]).c_str());
}
}

void nodes::getPublications()
{
/* getPublications(caller_id): Retrieve a list of topics that this node publishes 
   Parameters:
      caller_id (str)
          ROS caller ID. 
    Returns (int, str, [ [str, str] ])
      (code, statusMessage, topicList)
                            topicList -> [ [topic1, topicType1]...[topicN, topicTypeN] ]
*/

  std::string peer_host = this->getHost();
  uint32_t peer_port = this->getPort();
  XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
  XmlRpc::XmlRpcValue req;
  XmlRpc::XmlRpcValue resp;
  req[0] = this->node_name;
  c.execute("getPublications", req, resp);
  
  if (!c.isFault() && resp.valid() && resp.size() > 0 && static_cast<int>(resp[0]) == 1)
  {
    for(int i = 0; i < resp[2].size(); i++)
    {
      std::string topic_name = static_cast<std::string>(resp[2][i][0]);
      std::string dataType =  static_cast<std::string>(resp[2][i][1]);
      if (topic_name != "/rosout")
      {
        auto dtmap = publishedTopics.find(topic_name);
        if (dtmap == publishedTopics.end())
        {
          rosTopic *topic = new rosTopic(nh_, priv_nh_);
          topic->topicName = topic_name;
          topic->dataType = dataType;
          topic->subscribeToTopics(topic_name, topic->hz);
          publishedTopics[topic_name] = topic;            
        }
      }
      ROS_INFO(" topic: %s  topictype: %s ", static_cast<std::string>(resp[2][i][0]).c_str(),  static_cast<std::string>(resp[2][i][1]).c_str());
    }
  }
  else
  {
   ROS_ERROR("Node at: [%s] failed to return subscriptions.", static_cast<std::string>(resp[1]).c_str());
 }
}

void rosTopic::subscribeToTopics(std::string topic_name, rostopicHZ &topic)
{
  ROS_INFO(" SUBSCRIBING TO %s ...........", topic_name.c_str());
  sub.push_back(nh_.subscribe(topic_name, 5000, &rostopicHZ::msg_callback, &topic, ros::TransportHints().tcpNoDelay()));
  wtimer.push_back(nh_.createWallTimer(ros::WallDuration(1.0), &rostopicHZ::status_callback, &topic));
}

void memoryChecker(dashboard *pointer)
{

  struct statvfs sbuf;
  long long int final_, initial;
  double diff = 0;

  memory_status *memStatus = pointer->memStatus;
  memStatus[0].setStoragePath(pointer->path1);
  memStatus[1].setStoragePath(pointer->path2);

  ROS_INFO(" PATH : %s ", memStatus[0].getStoragePath().c_str());
  ROS_INFO(" PATH : %s ", memStatus[1].getStoragePath().c_str());

  memset(&sbuf, 0, sizeof(sbuf));
  if (statvfs(memStatus[0].getStoragePath().c_str(), &sbuf) < 0)
   exit(-1);
 memStatus[0].total_size = sbuf.f_blocks * sbuf.f_frsize;

 memset(&sbuf, 0, sizeof(sbuf));
 if (statvfs(memStatus[1].getStoragePath().c_str(), &sbuf) < 0)
   exit(-1);
 memStatus[1].total_size = sbuf.f_blocks * sbuf.f_frsize;

 do
 {
  for (int i=0; i < MAX_SSDS_CONNECTED; i++)
  {

    memset(&sbuf, 0, sizeof(sbuf));
    if (statvfs(memStatus[i].getStoragePath().c_str(), &sbuf) < 0)
      exit(-1);

    initial = sbuf.f_bsize * sbuf.f_bavail;
    sleep(1);

    memset(&sbuf, 0, sizeof(sbuf));
    if (statvfs(memStatus[i].getStoragePath().c_str(), &sbuf) < 0)
      exit(-1);

    final_ = sbuf.f_bsize * sbuf.f_bavail;

    ROS_DEBUG(" Initial - %lld and final - %lld  and final - initial : %lld ", initial, final_, initial - final_); 

    if (initial - final_)
     diff = ceil (initial / (initial - final_));

   memStatus[i].memory_available_ = final_;
   memStatus[i].seconds_left_ = (initial == final_) ? " Disk un utilized: zzzz secs.. ! " : std::to_string(diff);
   
   unsigned long long int freespace = sbuf.f_bsize * sbuf.f_bavail;
   ROS_DEBUG(" filesystem block size : %ld\n",sbuf.f_bsize);
   ROS_DEBUG(" filesystem blocks that are free : %ld\n", sbuf.f_bfree * sbuf.f_bsize);
   ROS_DEBUG("  FOR DISK : %s Total Memory free - %lld :: secs left: %lf " ,memStatus[i].getStoragePath().c_str(), final_, diff);
   ROS_DEBUG(" inode size - %ld \n ", sbuf.f_files);      
 }
 ROS_INFO(" DISK : %s  Available : %lld  Secs left to fill : %s", memStatus[0].getStoragePath().c_str(), memStatus[0].memory_available_,  memStatus[0].seconds_left_.c_str());
 ROS_INFO(" DISK : %s  Available : %lld  Secs left to fill : %s", memStatus[1].getStoragePath().c_str(), memStatus[1].memory_available_,  memStatus[1].seconds_left_.c_str());
 pointer->mqtt_client->send_memory_status(memStatus);

}while(  (memStatus[0].memory_available_ > MAX_MEM_LEVEL_TO_WARN) || ( memStatus[1].memory_available_) > MAX_MEM_LEVEL_TO_WARN);
ROS_ERROR(" ONE OF THE MEMORY DISK GOT FILLED ");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dashboard");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  mosqpp::lib_init();

  if (gethostname(hostbuffer, sizeof(hostbuffer)) == -1 )
  {
   perror("getHostName");
   return -1;
 }
 std::string hostString(hostbuffer);
 std::string mqttClientName = hostString + "dashboardClient";
 dashboard dboard(nh, priv_nh);

 dboard.mqtt_client = new mqttClient(mqttClientName.c_str(), "localhost", 1883);
 dboard.mqtt_client->loop_start();
 if (!dboard.mqtt_client->start)
 {
  dboard.mqtt_client->waitForMqttStart();
}

dboard.createTimer();

std::thread th1(memoryChecker, &dboard);

ros::Rate loop_rate(100);
while (ros::ok())
{
  ros::spinOnce();
  loop_rate.sleep();
}
dboard.mqtt_client->loop_stop();

th1.join();
return 0;	
}
