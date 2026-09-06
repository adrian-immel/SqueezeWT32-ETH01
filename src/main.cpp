#include <Arduino.h>
#include "config.h"

#include <ETH.h>
#include <WiFi.h>     // provides the WiFiClient class used over Ethernet
#include <WiFiUdp.h>  // WiFiUDP = NetworkUDP (works over Ethernet)

#include "slimproto.h"

/*
 * WT32-ETH01 Ethernet : LAN8720A PHY, RMII with an external 50MHz oscillator
 * fed into GPIO0. GPIO16 must be driven high to enable that oscillator (the
 * Arduino ETH lib does it through the "power" pin argument). MDC/MDIO are
 * GPIO23/GPIO18 and the PHY address is 1.
 * See https://github.com/egnor/wt32-eth01
 *
 * NOTE : this is the Arduino-ESP32 >= 3.1.0 argument order (pioarduino
 * platform = core 3.3.11) :
 *   begin(phy_type, phy_addr, mdc, mdio, power, clk_mode)
 * On the older core 3.0.x it was
 *   begin(phy_addr, power, mdc, mdio, type, clk_mode)
 */
#define ETH_PHY_ADDR   1
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_PHY_POWER  16

WiFiClient     client;            // TCP connection to the LMS control port
WiFiUDP        udp;               // Used for the LMS autodiscovery broadcast
slimproto *    vislimCli = 0;

int       viCnxAttempt = -1;      // -1 => ask LMS_addr to be reset / rediscovered
IPAddress LMS_addr(0, 0, 0, 0);

/**
 * Block until the PHY link is up and DHCP has given us an address
 */
void waitForEthernet()
{
  while(!ETH.linkUp())
  {
    Serial.println("Waiting for Ethernet link ...");
    delay(500);
  }
  Serial.println("Ethernet link is up");

  while(ETH.localIP() == IPAddress(0, 0, 0, 0))
  {
    Serial.print("Waiting for IP address .");
    while(ETH.localIP() == IPAddress(0, 0, 0, 0))
    {
      Serial.print(".");
      delay(500);
    }
    Serial.println();
  }
  Serial.print("IP address : ");
  Serial.println(ETH.localIP());
  Serial.print("Ethernet MAC : ");
  Serial.println(ETH.macAddress());
}

/**
 * Slimproto UDP autodiscovery : broadcast 'e', the server answers 'E' with
 * its address. Returns true and fills LMS_addr when a server is found.
 */
bool discoverLMS()
{
  for(int nbSend = 0; nbSend < 10; nbSend++)
  {
    udp.flush();
    udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT);
    udp.printf("e");
    udp.endPacket();
    Serial.print("Discovery packet sent (");
    Serial.print(nbSend + 1);
    Serial.println("/10)");

    unsigned long viStart = millis();
    while(millis() - viStart < 2500)
    {
      if(udp.parsePacket() > 0)
      {
        char viPacket = udp.read();
        if(viPacket == 'E')
        {
          LMS_addr = udp.remoteIP();
          Serial.print("Found LMS server @ ");
          Serial.println(LMS_addr);
          return true;
        }
      }
      delay(10);
    }
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("SqueezeWT32-ETH01 - Squeezebox player");
  Serial.printf("Compiled %s %s\n", __DATE__, __TIME__);
  Serial.printf("Free heap at boot : %u bytes\n", ESP.getFreeHeap());

  ETH.begin(ETH_PHY_LAN8720, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            ETH_PHY_POWER, ETH_CLOCK_GPIO0_IN);
  waitForEthernet();

  udp.begin(UDP_PORT);
}

void loop()
{
  // Make sure the wired link is usable (covers boot and cable unplugged)
  if(!ETH.linkUp() || ETH.localIP() == IPAddress(0, 0, 0, 0))
  {
    Serial.println("Ethernet not ready");
    waitForEthernet();
    LMS_addr = IPAddress(0, 0, 0, 0);
    viCnxAttempt = -1;
  }

  // Erase the LMS address after too many failed connection attempts
  if(viCnxAttempt == -1)
    LMS_addr = IPAddress(0, 0, 0, 0);

  // If no LMS known yet, try to autodiscover it
  if(LMS_addr[0] == 0)
  {
    if(!discoverLMS())
    {
      Serial.println("No LMS server found, try again in 10 seconds");
      delay(10000);
      return;
    }
  }

  Serial.print("Connecting to server @ ");
  Serial.println(LMS_addr);

  viCnxAttempt++;

  if(!client.connect(LMS_addr, LMS_PORT))
  {
    Serial.println("Connection failed, pause and try connect...");

    viCnxAttempt++;
    if(viCnxAttempt > 30)
      viCnxAttempt = -1;        // Will erase LMS addr in the next attempt

    delay(2000);
    return;
  }

  viCnxAttempt = 0;

  if(vislimCli)
    delete vislimCli, vislimCli = 0;

  vislimCli = new slimproto(LMS_addr.toString(), &client);

  Serial.println("Connection Ok, send hello to LMS");
  uint8_t viMac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
  ETH.macAddress(viMac);

  reponseHelo * HeloRsp = new reponseHelo(&client, viMac);
  HeloRsp->sendResponse();
  delete HeloRsp;

  while(client.connected())
  {
    if(!ETH.linkUp())
      break;

    if(!vislimCli->HandleMessages())
      break;

    vislimCli->HandleAudio();
  }

  Serial.println("Disconnected from LMS, reconnecting...");
  client.stop();

  if(vislimCli)
    delete vislimCli, vislimCli = 0;
}
