import sys

def main():
    raw_data = sys.argv
    
    if len(raw_data) != 2:
        print("usage: %s instance" % raw_data[0])
        sys.exit(1)
    
    instance = raw_data[1]
    
    with open(instance, 'r') as file:
        data = file.readlines()
        
    width, height = list(map(int, data[0].split()))
    grid = list(map(lambda l: list(l.rstrip('\n')), data[1:]))

if __name__ == '__main__':
    main()