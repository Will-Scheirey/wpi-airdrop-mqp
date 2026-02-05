import csv

base_dir = '/Users/paigerust/Downloads/weather'
data_dir_1 = '/2025-08-07'
data_dir_2 = '/2025-08-05'

input = base_dir + data_dir_2 + '/08052100Z.txt'
output = base_dir + data_dir_2 + '/08052100Z.csv'

delimiter = ','
header = ['alt_agl', 'pressure', 'direction', 'speed', 'temperature', 'alt_pres_msl', 'd_value', 'wind_direction', 'win_speed']

try:
    with open(input, 'r', newline='', encoding='utf-8') as in_file:
        # Create a reader object, specifying the input file's delimiter
        reader = csv.reader(in_file, delimiter=delimiter)
        
        with open(output, 'w', newline='', encoding='utf-8') as out_file:
            # Create a writer object for the CSV output
            writer = csv.writer(out_file)

            writer.writerow(header)
            
            # Write all rows from the reader to the writer
            for row in reader:
                writer.writerow(row)
    print(f"Successfully converted {input} to {output}")

except FileNotFoundError:
    print(f"Error: {input} not found.")
except Exception as e:
    print(f"An error occurred: {e}")

