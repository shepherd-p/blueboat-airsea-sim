import QtQuick 2.9
Item {
  property bool sensorMain: true
  width: 1280
  height: 720
  Rectangle { anchors.fill: parent; color: "#13232d" }
  Image { anchors.fill: parent; fillMode: Image.PreserveAspectFit; cache: false; source: "image://airsea/" + (sensorMain ? "sensor" : "chase") + "?" + AirSeaCapture.frameId }
  Rectangle {
    anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.margins: 12
    width: parent.width * 0.30; height: width * 9/16; color: "#13232d"; border.color: "#c2d3dd"; border.width: 2
    Image { anchors.fill: parent; anchors.margins: 2; fillMode: Image.PreserveAspectFit; cache: false; source: "image://airsea/" + (sensorMain ? "chase" : "sensor") + "?" + AirSeaCapture.frameId }
    MouseArea { anchors.fill: parent; onClicked: sensorMain = !sensorMain }
  }
  Rectangle {
    anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 12
    width: 145; height: 34; radius: 4; color: "#263c49"
    Text { anchors.centerIn: parent; text: "Swap camera views"; color: "white" }
    MouseArea { anchors.fill: parent; onClicked: sensorMain = !sensorMain }
  }
}
