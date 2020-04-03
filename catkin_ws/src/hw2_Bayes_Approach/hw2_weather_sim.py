import numpy as np 

def weather_simulation(M, n_days = 5):
	'''
	args: 
		M: 3x3 transition_matrix, weather transition matrix (Markov chain)
		n_days: number of days you want to simulate weather

	return: w: 3x1 weather vector
	'''
	w = np.zeros((3, 1))

	# Day 1: Sunny
	w[0] = 1
	print('\tSunny \tCloudy \tRainy')

	for i in range(n_days):
		w = np.dot(M, w)
		print('Day{}: \t{:.3f} \t{:.3f} \t{:.3f}'.format((i+1), w[0][0], w[1][0], w[2][0]))


def main():
	#M = np.array([[.8, .4, .2], [.2, .4, .6], [0, .2, .2]])
	M = np.array([[.6, .3, 0], [.4, .7, 0], [0, 0, 1]])

	weather_simulation(M, 5)

if __name__ == '__main__':
	main()