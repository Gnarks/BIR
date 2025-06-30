import math

# parameters, entering nothing will take the default value (after or)
# times are in ms and bw in Hz
number_of_device = int(input('Number of devices (default=15): ').strip() or "15")
number_of_frame_per_device = int(input('number_of_frame_per_device (default=1000): ').strip() or "1000")
spreding_factor = int(input('spreading factor (default=7): ').strip() or "7")
time_between_frame = int(input('time_between_frame (default=20): ').strip() or "20")
bandwith = int(input('bandwith (default=125000): ').strip() or "125000")
preambule_size = int(input('preambule_size (default=8): ').strip() or "8")

single_frame_transimttion_time= round( (preambule_size +4.25)* (math.pow(2,spreding_factor)/bandwith) * 1000, 2 )

full_cycle_of_transmittion = round((single_frame_transimttion_time +time_between_frame) * number_of_device,4 )

total_time = round(full_cycle_of_transmittion * number_of_frame_per_device, 4)

print("===========================================")

print(f"single_frame_transimttion_time : {single_frame_transimttion_time} ms")
print(f"full_cycle_of_transmittion : {full_cycle_of_transmittion} ms")

print(f"total time : {total_time} ms")
print(f"total time : {total_time/1000} s")
print(f"total time : {total_time / 60000} mins")
print(f"total time : {total_time / 3600000} hours")

