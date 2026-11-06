import QtQuick 2.9
Item {
  width: 1280
  height: 720
  Rectangle { anchors.fill: parent; color: "#13232d" }
  Image { anchors.fill: parent; fillMode: Image.PreserveAspectFit; cache: false; source: "image://airsea/live?" + AirSeaCapture.frameId }
}
