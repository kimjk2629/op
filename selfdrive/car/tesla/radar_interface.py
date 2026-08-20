from selfdrive.car.interfaces import RadarInterfaceBase


class RadarInterface(RadarInterfaceBase):
  # Radar support (Continental ARS4-B on some 2017-2021 Model 3/Y) was not carried
  # over in this initial port. ret.radarUnavailable=True in interface.py means this
  # class's update() (inherited from RadarInterfaceBase) is effectively a no-op stub.
  pass
