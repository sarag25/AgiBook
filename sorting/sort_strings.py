def sort_strings(strings):
    return sorted(strings, key=str.lower)


if __name__ == "__main__":
    book_list = ["It", "Hunger Games", "Emma"]
    print("Prima:", book_list)
    print("Dopo: ", sort_strings(book_list))