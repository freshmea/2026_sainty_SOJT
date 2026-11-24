def main():
    print("Hello from python-test!")
    # python version
    print(f"Python version: {'.'.join(map(str, __import__('sys').version_info[:3]))}")


if __name__ == "__main__":
    main()
