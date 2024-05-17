import commLaptop

COMMANDLIST = {"move", "arm", "power"}

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
        words=fullCommand.split()
        command = words[0]
        arguments = words[1:]

        # Exit command
        if command == "exit":
            exit=True
        elif command in COMMANDLIST:
            print("Executing command "+ command)
            print("Arguments")
            print(arguments)
            client.send_message(fullCommand)

        else:
            print("Unknown Command")


    print("Exiting")
    client.close()

if __name__ == "__main__":
    main()