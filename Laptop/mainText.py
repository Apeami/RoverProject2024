import commLaptop



def main():
    ip = input("Enter target IP Address:\n")
    try:
        port = int(input("Enter Port:\n"))
    except:
        print("Error in inputting")

    client = commLaptop.Client(ip,port)

    res = client.connect()
    if res == False:
        return
    print("Succesfully connected to the PI")

    # This is the infinate command loop that you can use to send commands
    exit=False
    while not exit:
        fullCommand = input(">")

        exit = client.execute_command(fullCommand)


if __name__ == "__main__":
    main()