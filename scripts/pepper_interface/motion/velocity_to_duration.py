
def velocity_to_duration(target_values, current_values, velocity):

	maximum = float("-inf")

	for joint, value in target_values.iteritems():

		try:
                        current = current_values[joint]
		except:
                        current = None

		if current is not None:
                        diff = abs(value - current)
			if diff > maximum:
				maximum = diff

	duration = maximum / abs(velocity)
	return duration
