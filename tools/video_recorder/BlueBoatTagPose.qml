import QtQuick 2.9
import QtQuick.Controls 2.2
Item {
  width: 1280
  height: 720
  Rectangle { anchors.fill: parent; color: "#101c25" }
  Image {
    anchors.fill: parent
    fillMode: Image.PreserveAspectFit
    cache: false
    source: "image://blueboattag/live?" + BlueBoatTagPose.frameId
  }
}
