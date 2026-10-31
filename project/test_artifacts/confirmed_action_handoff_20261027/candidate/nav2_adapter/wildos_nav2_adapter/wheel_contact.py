"""Classify measured wheel support without treating wheel RPM as vehicle motion."""
import math


WHEEL_SNAPSHOT_LUA = '''
local out={}
for _,w in pairs(wheels.wheels or {}) do
  local data={name=w.name,isPropulsed=w.isPropulsed}
  for _,key in ipairs({'wheelSpeed','downForceRaw','contactMaterialID1',
      'contactMaterialID2','brakingTorque','propulsionTorque'}) do
    local value=w[key]
    if type(value)=='number' and value==value and value~=math.huge and value~=-math.huge then
      data[key]=value
    end
  end
  table.insert(out,data)
end
return jsonEncode(out)
'''


def classify_wheel_contact(wheels, speed):
    if not isinstance(wheels, list) or not math.isfinite(speed):
        return None
    airborne, spinning = [], []
    valid = 0
    for wheel in wheels:
        if not isinstance(wheel, dict):
            continue
        values = [wheel.get(k) for k in ('downForceRaw', 'wheelSpeed',
                  'contactMaterialID1', 'contactMaterialID2')]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            continue
        valid += 1
        force, rim_speed, material1, material2 = values
        if force <= 5. and material1 < 0 and material2 < 0:
            airborne.append(str(wheel.get('name', '?')))
            if abs(rim_speed) > .3 and speed < .1:
                spinning.append(str(wheel.get('name', '?')))
    if not valid:
        return None
    return dict(classification='wheel_airborne_spin' if spinning else
                'wheel_support_lost' if airborne else 'wheel_support_present',
                airborne_wheels=airborne, spinning_airborne_wheels=spinning,
                measured_wheels=valid)
