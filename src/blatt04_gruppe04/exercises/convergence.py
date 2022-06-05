
prob_rain_day_2 = 621/703
prob_norain_day_2 = 82/703

last_prob_rain = prob_rain_day_2
last_prob_norain = prob_norain_day_2

for i in range(3, 1000):
    a_priori_rain = 0.7 * last_prob_rain + 0.3 * last_prob_norain
    a_priori_norain = 0.3 * last_prob_rain + 0.7 * last_prob_norain
    
    # With Umbrella
    a_posteriori_rain = 0.9 * a_priori_rain
    a_posteriori_norain = 0.2 * a_priori_norain
    
    # normierung
    eta = 1 / (a_posteriori_rain + a_posteriori_norain)
    
    a_posteriori_rain = a_posteriori_rain * eta
    a_posteriori_norain = a_posteriori_norain * eta
    
    # set for next iteration
    
    last_prob_rain = a_posteriori_rain
    last_prob_norain = a_posteriori_norain

print("Task: e")    
print("i:", i, "a_post rain:", a_posteriori_rain, "a_post sun:", a_posteriori_norain)


last_prob_rain = prob_rain_day_2
last_prob_norain = prob_norain_day_2

for i in range(3, 1000):
    a_priori_rain = 0.7 * last_prob_rain + 0.3 * last_prob_norain
    a_priori_norain = 0.3 * last_prob_rain + 0.7 * last_prob_norain
    
    # No Director: a_posteriori = a_priori
    a_posteriori_rain = a_priori_rain
    a_posteriori_norain = a_priori_norain
    
    # normierung: eta = 1
    eta = 1 / (a_posteriori_rain + a_posteriori_norain)

    a_posteriori_rain = a_posteriori_rain * eta
    a_posteriori_norain = a_posteriori_norain * eta
    
    # set for next iteration
    
    last_prob_rain = a_posteriori_rain
    last_prob_norain = a_posteriori_norain

print("Task: f")    
print("i:", i, "a_post rain:", a_posteriori_rain, "a_post sun:", a_posteriori_norain)