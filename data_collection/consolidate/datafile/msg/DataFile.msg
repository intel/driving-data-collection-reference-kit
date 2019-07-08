Header header

string host 				# To represent the system, could be the network ip

string uuid 				# To represent the disk universal unique id

string filepath 			# Relative path of the file including name of file

int64 file_offset_begin 	# Start offset of data, default "0" useful to track multiple frames dumped to single file

int64 file_offset_end 		# End offset of data, default size of frame, useful to track multiple frames dumped to a single file

string hash 				# Check for file integrity

