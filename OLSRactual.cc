#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mmwave-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/csma-module.h"
#include "ns3/lte-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/olsr-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/ssid.h"
#include "ns3/random-waypoint-mobility-model.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/basic-energy-source.h"
#include "ns3/wifi-radio-energy-model.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/energy-source-container.h"
#include "ns3/device-energy-model-container.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/energy-module.h"
#include "ns3/buildings-helper.h"

using namespace ns3;
using namespace mmwave;

NS_LOG_COMPONENT_DEFINE ("OlsrHna");

class RoutingExperiment
 {
public:
 RoutingExperiment ();
  void Run (int nSinks, double txp, std::string CSVfileName);
  //static void SetMACParam (ns3::NetDeviceContainer & devices,
  //                                 int slotDistance);
  std::string CommandSetup (int argc, char **argv);

private:
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
  void ReceivePacket (Ptr<Socket> socket);
  void CheckThroughput ();

  uint32_t port;
  uint32_t bytesTotal;
  uint32_t packetsReceived;

  std::string m_CSVfileName;
  int m_nSinks;
  std::string m_protocolName;
  double m_txp;
  bool m_traceMobility;
  uint32_t m_protocol;
 };

RoutingExperiment::RoutingExperiment ()
  : port (9),
    bytesTotal (0),
    packetsReceived (0),
    m_CSVfileName ("OLSR.csv"),
    m_traceMobility (false),
    m_protocol (1) 
{
}

static inline std::string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (senderAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
      oss << " received one packet from " << addr.GetIpv4 ();
    }
  else
    {
      oss << " received one packet!";
    }
  return oss.str ();
}



void
RoutingExperiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
      bytesTotal += packet->GetSize ();
      packetsReceived += 1;
      NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
    }
}

void
 RoutingExperiment::CheckThroughput ()
 {
   double kbs = (bytesTotal * 8.0) / 1000;
   bytesTotal = 0;
 
   std::ofstream out (m_CSVfileName.c_str (), std::ios::app);
 
   out << (Simulator::Now ()).GetSeconds () << ","
       << kbs << ","
       << packetsReceived << ","
       << m_nSinks << ","
       << m_protocolName << ","
       << m_txp << ""
       << std::endl;
 
   out.close ();
   packetsReceived = 0;
   Simulator::Schedule (Seconds (1.0), &RoutingExperiment::CheckThroughput, this);
 }

Ptr<Socket>
RoutingExperiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));
  
  return sink;
}

std::string
RoutingExperiment::CommandSetup (int argc, char **argv)
{
  CommandLine cmd;
  cmd.AddValue ("CSVfileName", "The name of the CSV output file name", m_CSVfileName);
  cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.Parse (argc, argv);
  return m_CSVfileName;
}

/*void RemainingEnergy (double oldValue, double remainingEnergy)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds () <<"s current remaining Energy " << remainingEnergy << "J");
}

void TotalEnergy (double oldValue, double totalEnergy)
 {
  if (Simulator::Now ().GetSeconds () > 0.5)
  {
   NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                  << "s Total energy consumed by radio = " << totalEnergy << "J");
  }
 }*/

int main (int argc, char *argv[])
{
  
  RoutingExperiment experiment;
  std::string CSVfileName = experiment.CommandSetup (argc,argv);
  
  std::ofstream out (CSVfileName.c_str ());
  out << "SimulationSecond," <<
  "ReceiveRate," <<
  "PacketsReceived," <<
  "NumberOfSinks," <<
  "RoutingProtocol," <<
  "TransmissionPower" <<
  std::endl;
  out.close ();

  int nSinks = 5;
  double txp = 16;

  experiment.Run (nSinks, txp, CSVfileName);

}

void RoutingExperiment::Run(int nSinks, double txp, std::string CSVfilename)
{
  Packet::EnablePrinting ();
  m_nSinks = nSinks;
  m_txp = txp;
  m_CSVfileName = CSVfilename;
  
  //get system time
  time_t ctime=time(0);
  uint32_t t=static_cast<uint32_t>(ctime);

  //set the seed value
  RngSeedManager::SetSeed(t);
  
  int nWifis = 30;
  int mcs = 9;
  int channelWidth = 80;
  int sgi = 0;
  
  double TotalTime = 2.0;
  //std::string phyMode ("DsssRate1Mbps");
  std::string phyMode ("VhtMcs7");
  std::string rate ("1000000bps");
  std::string tr_name ("OLSRact");
  double rss = -45;  // -dBm
  int nodeSpeed = 10;
  int nodePause = 0;

  Config::SetDefault ("ns3::MmWavePhyMacCommon::SubframePeriod" , DoubleValue (100.0));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue(100));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ResourceBlockNum", UintegerValue(1));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::SlotsPerSubframe" , UintegerValue (32));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkWidth", DoubleValue (13.889e6));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (28e9));
  //Config::SetDefault ("ns3::MmwavePhyMacCommon::TDDPattern", StringValue ("ccdddddd"));
  //Config::SetDefault ("ns3::MmwavePhyMacCommon::UlSchedDelay", UintegerValue (1));
  //Config::SetDefault ("ns3::MmwavePhyMacCommon::NumRbPerRbg", UintegerValue (2));
  //Config::SetDefault ("ns3::MmwavePhyMacCommon::WbCqiPeriod", DoubleValue (1500.0));
  //Config::SetDefault ("ns3::MmwavePhyMacCommon::GuardPeriod", DoubleValue (4.16));
  Config::SetDefault ("ns3::MmWavePropagationLossModel::FixedLossTst", BooleanValue (false));
  Config::SetDefault ("ns3::MmWavePropagationLossModel::LossFixedDb", DoubleValue (100.0));
  Config::SetDefault ("ns3::OnOffApplication::PacketSize", StringValue("1472"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue(rate));

  Ptr<MmWaveHelper> ptr_mmwave = CreateObject<MmWaveHelper> ();
  ptr_mmwave->Initialize();

  NodeContainer olsrNodes;
  olsrNodes.Create (nWifis);

  NodeContainer enbNodes;
  enbNodes.Create(nSinks);

  Ptr<ListPositionAllocator> enbPos = CreateObject<ListPositionAllocator> ();
  /*enbPos->Add (Vector (10.5, 5.0, 4.5));
  enbPos->Add (Vector (8.0, 5.0, 5.5));
  enbPos->Add (Vector (4.0, 1.0, 5.0));
  enbPos->Add (Vector (7.0, 8.0, 9.5));
  enbPos->Add (Vector (7.8, 10.0, 6.0));*/
enbPos->Add (Vector (4.5, 5.0, 4.5));
  enbPos->Add (Vector (5.0, 5.0, 4.5));
  enbPos->Add (Vector (4.0, 5.0, 5.0));
  enbPos->Add (Vector (4.0, 5.0, 4.5));
  enbPos->Add (Vector (4.0, 4.0, 4.0));
  //enbPos->Add (Vector (4.0, 4.0, 4.0));
  //enbPos->Add (Vector (4.0, 4.0, 4.0));
  //enbPos->Add (Vector (4.0, 5.0, 4.0));
  //enbPos->Add (Vector (4.0, 4.0, 5.5));
  //enbPos->Add (Vector (3.5, 5.0, 4.0));
  //enbPos->Add (Vector (4.3, 4.5, 5.0));
 // enbPos->Add (Vector (4.4, 4.5, 2.0));
  //enbPos->Add (Vector (4.7, 2.4, 3.1));
    
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPos);
  enbmobility.Install (enbNodes);
  
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  //wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_PRISM_HEADER);
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
  WifiMacHelper wifiMac;
  
  std::ostringstream oss;
  oss << "DsssRate1Mbps" << mcs;

  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));
  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));

  Ssid ssid = Ssid("ns3-80211ac");
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, olsrNodes);
  //NetDeviceContainer enbDevice = ptr_mmwave->InstallEnbDevice (olsrNodes);  
  NetDeviceContainer devices2 = wifi.Install (wifiPhy, wifiMac, enbNodes);  


  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (channelWidth));
  // Set guard interval
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported", BooleanValue (sgi));

  MobilityHelper mobility;
  
  int64_t streamIndex = 0; //used to get consistent ,mobility accross scenarios

  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));


  Ptr<PositionAllocator> positionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  streamIndex += positionAlloc->AssignStreams (streamIndex);
  
  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable [Min=0.0|Max=" << nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
  
  mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel", "Speed",
  StringValue (ssSpeed.str()), "Pause",
  StringValue (ssPause.str()), "PositionAllocator",
  PointerValue(positionAlloc));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (olsrNodes);
  streamIndex += mobility.AssignStreams (olsrNodes, streamIndex);
  NS_UNUSED (streamIndex);  

   Ptr <LteHelper> lteHelper = CreateObject<LteHelper> ();
  NetDeviceContainer ueDevice = ptr_mmwave->InstallUeDevice (olsrNodes); //ue device
  NetDeviceContainer enbDevice = ptr_mmwave->InstallEnbDevice (enbNodes); //enb device

  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (26));
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (36));
  //ptr_mmwave->EnableTraces ();
  enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
  EpsBearer bearer (q);

  //ptr_mmwave->AttachToClosestEnb (ueDevice, enbDevice);
  //ptr_mmwave->ActivateDataRadioBearer (ueDevice, bearer);
  OlsrHelper olsr;
  Ipv4ListRoutingHelper list;

  InternetStackHelper internet_olsr;
  list.Add (olsr, 100);
  m_protocolName = "OLSR";

  internet_olsr.SetRoutingHelper (list); 
  internet_olsr.Install (olsrNodes);
  BuildingsHelper::Install (olsrNodes);
  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = ipv4.Assign (devices);
  
  //Ipv4ListRoutingHelper lt;        //For enb nodes
  //InternetStackHelper io;
  //lt.Add (olsr, 100);
  
  internet_olsr.SetRoutingHelper (list);
  internet_olsr.Install (enbNodes);
  Ipv4AddressHelper ipv42;
  ipv42.SetBase ("12.1.1.0", "255.255.255.0");         //For the enb nodes
  Ipv4InterfaceContainer adjoc;  
  adjoc = ipv42.Assign (devices2);

  //ptr_mmwave->AttachToClosestEnb (ueDevice, enbDevice);
  //ptr_mmwave->ActivateDataRadioBearer (ueDevice, bearer);  

  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());    //  "ns3::TcpSocketFactory", Ipv4Address::GetAny ()
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

  //ptr_mmwave->AttachToClosestEnb (ueDevice, enbDevice);
  //ptr_mmwave->ActivateDataRadioBearer (ueDevice, bearer);
  
  for (int i = 0; i < nSinks; i++)
  {
      Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (i), olsrNodes.Get (i));

      AddressValue remoteAddress (InetSocketAddress (adhocInterfaces.GetAddress (i), port));
      onoff1.SetAttribute ("Remote", remoteAddress);

      //Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
      ApplicationContainer temp = onoff1.Install (olsrNodes.Get (i + nSinks));
      //temp.Start (Seconds (var->GetValue (100.0,101.0)));
      temp.Start (Seconds (0.4));
      temp.Stop (Seconds (TotalTime + 0.5));
  }

  ptr_mmwave->AttachToClosestEnb (ueDevice, enbDevice);
  ptr_mmwave->ActivateDataRadioBearer (ueDevice, bearer);

  ptr_mmwave->EnableTraces ();
  AsciiTraceHelper ascii;
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();
  
  CheckThroughput();
  
  AnimationInterface anim ("anim6.xml");
  //anim.SetMaxPktsPerTraceFile(1000);
  Simulator::Stop (Seconds (TotalTime + 0.5));
  Simulator::Run ();
  
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

  /*Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();*/
  
  int totaltxpacket = 0, totalrxpacket = 0, lostpackets;  
    double totalthr = 0.0, delay_total, plr = 0.0, totaldelay = 0.0;
    int count = 0;
    int hop_count = 0;
    double pl = 0.0;

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator p = stats.begin (); p != stats.end (); ++p)
     {
       count ++;
          Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (p->first);

          std::cout << "Flow " << p->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
         
           NS_LOG_UNCOND("  Tx Packets: "<<p->second.txPackets);
           NS_LOG_UNCOND("  Rx Packets: "<<p->second.rxPackets);

           
	   totaldelay += p->second.delaySum.GetSeconds();
	 
           totaltxpacket = totaltxpacket + p ->second.txPackets;
           totalrxpacket = totalrxpacket + p ->second.rxPackets;         
                   
              
           if (p ->second.rxPackets > 0)
           {
             NS_LOG_UNCOND("  Hop count: "<< p->second.timesForwarded / p->second.rxPackets + 1);
             //NS_LOG_UNCOND("  Hop count: "<< p->second.timesForwarded + 1);
             hop_count += p->second.timesForwarded / p->second.rxPackets + 1;   
           }     
     }
     if(totalrxpacket!=0)
     {
	   delay_total = ((double) totaldelay / (double) totalrxpacket) * 1000;
     } 
      
   totalthr = totalrxpacket * 1472.0 * 8.0 / ((TotalTime + 0.5) * 1000.0); // 1472 bytes packet size, Total packet transmission    time=TotalTime  
   Simulator::Destroy ();

   //count lost packets
   lostpackets = totaltxpacket - totalrxpacket;   
   if (totaltxpacket != 0)
   {
   pl = (double)lostpackets / totaltxpacket;
   plr = pl * 100.0;
   }
  
   plr = plr;
   delay_total = delay_total / double (nSinks);

   std::cout << "Overall avg throughput = "<< totalthr<<"Kbps"<<std::endl;
   totalthr = totalthr / count;
   std::cout << "Per-node avg throughput = "<< totalthr<<"Kbps"<<std::endl;
   std::cout << "Packet loss rate = "<< plr<<"%"<<std::endl;
   std::cout << "Packet delay = "<< delay_total<<"ms"<<std::endl;
   std::cout << "Hop count = "<< hop_count<<std::endl;
   std::cout << "Lost Packets = "<<lostpackets<<std::endl;

   monitor->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), false, false);

}
