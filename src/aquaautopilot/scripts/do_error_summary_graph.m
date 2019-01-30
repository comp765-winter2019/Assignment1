close all;

tag = 'Flat Swimming';
[r,p,y,d] = compute_tracking_errors( 200, 219, tag, '/media/passport/AquaData/ship_programming/Jan15_last_bbds_jolly_rogers//uberpilot__2014-01-15-15-01-08' );
fprintf(1,'Motion: %s. Errors roll %f, pitch %f, yaw %f, depth %f.\n',tag,r,p,y,d);

tag = 'KnifeEdge';
[r,p,y,d] = compute_tracking_errors( 3, 15, tag, '/media/passport/AquaData/aqua_autopilot_data/Jan10_second_more_choreo_depth_d_searches/uberpilot__2014-01-10-17-04-07' );
fprintf(1,'Motion: %s. Errors roll %f, pitch %f, yaw %f, depth %f.\n',tag,r,p,y,d);

%tag = '1stSnowplowWreck'
%[r,p,y,d] = compute_tracking_errors( 16, 29, tag, '/media/passport/AquaData/ship_programming/Jan15_last_bbds_jolly_rogers//uberpilot__2014-01-15-15-41-39' )
%tag = '2ndSnowplowWreck'
%[r,p,y,d] = compute_tracking_errors( 46, 59, tag, '/media/passport/AquaData/ship_programming/Jan15_last_bbds_jolly_rogers//uberpilot__2014-01-15-15-41-39' )
%tag = 'Jan10SquareDance1stHeave'
%[r,p,y,d] = compute_tracking_errors( 16, 29, tag, '/media/passport/AquaData/aqua_autopilot_data/Jan10_second_more_choreo_depth_d_searches/uberpilot__2014-01-10-17-04-07' )
%tag = 'Pitch p search section'
%[r,p,y,d] = compute_tracking_errors( 270, 335, tag, '/media/passport/AquaData/aqua_autopilot_data/Jan10_first_nice_choreo_pitch_and_depth_searches/uberpilot__2014-01-10-11-44-13' )

tag = 'Vertical Snowplow';
[r,p,y,d] = compute_tracking_errors( 140,152, tag, '/media/passport/AquaData/ship_programming/Jan15_last_bbds_jolly_rogers//uberpilot__2014-01-15-15-41-39' );
fprintf(1,'Motion: %s. Errors roll %f, pitch %f, yaw %f, depth %f.\n',tag,r,p,y,d);

tag = 'Circular Helix';
[r,p,y,d] = compute_tracking_errors( 120,155, tag, '/media/passport/AquaData/ship_programming/Jan15_last_bbds_jolly_rogers//uberpilot__2014-01-15-15-46-25' );
fprintf(1,'Motion: %s. Errors roll %f, pitch %f, yaw %f, depth %f.\n',tag,r,p,y,d);

tag = 'Square Helix at 36 degrees per second';
[r,p,y,d] = compute_tracking_errors( 180,189, tag, '/media/passport/AquaData/ship_programming/Jan15_last_bbds_jolly_rogers//uberpilot__2014-01-15-15-41-39' );
fprintf(1,'Motion: %s. Errors roll %f, pitch %f, yaw %f, depth %f.\n',tag,r,p,y,d);
