from cnoid.Base import *
from cnoid.BodyPlugin import *

SR1 = Item.find("SR1").body
SR1.link("LLEG_ANKLE_R").setMaterial("Foot")
SR1.link("RLEG_ANKLE_R").setMaterial("Foot")

Floor = Item.find("Floor").body
Floor.rootLink.setMaterial("Floor")
