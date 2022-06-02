-- script to detect GPS interference and disable based on EKF velocity innovation and
-- number of satellites

local GPS_disabled = false
local have_had_good_lock = false

function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

MAX_VEL_VARIANCE = bind_param("SCR_USER1")
MIN_SATELLITES = bind_param("SCR_USER2")
AHRS_GPS_USE = bind_param("AHRS_GPS_USE")

-- set reasonable defaults
if MAX_VEL_VARIANCE:get() <= 0.0 then
   MAX_VEL_VARIANCE:set(0.7)
end

if MIN_SATELLITES:get() <= 12 then
   MIN_SATELLITES:set(12)
end

-- the main update function
function check_ekf()

   local gps_primary = gps:primary_sensor()
   if not gps_primary then
      return
   end
   if gps:status(gps_primary) < gps.GPS_OK_FIX_3D then
     -- when no 3D fix the EKF can look after itself
     return
   end

   local num_sats = gps:num_sats(gps_primary)
   local velocity_variance = ahrs:get_variances()

   local ekf_ok = num_sats >= MIN_SATELLITES:get() and velocity_variance <= MAX_VEL_VARIANCE:get()
   if not have_had_good_lock then
      -- wait till all is first OK
      if ekf_ok then
         gcs:send_text(0, "Good Initial GPS and EKF")
         have_had_good_lock = true
      else
         return
      end
   end

   if not ekf_ok then
      if not GPS_disabled then
         GPS_disabled = true
         gcs:send_text(0, "Disabling EKF use of GPS")
         -- switch to EKF3 2nd source set, which should be setup without GPS
         ahrs:set_posvelyaw_source_set(1)
         -- also disable use of GPS by DCM
         AHRS_GPS_USE:set(0)
      end
   else
      if GPS_disabled then
         GPS_disabled = false
         gcs:send_text(0, "Enabling EKF use of GPS")
         -- switch to EKF3 1st source set, which should be setup with GPS
         ahrs:set_posvelyaw_source_set(0)
         -- also enable use of GPS by DCM
         AHRS_GPS_USE:set(1)
      end
   end

end

function update()
   check_ekf()
   return update, 1000
end

gcs:send_text(0, "Loaded GPS/EKF checking")

return update()
