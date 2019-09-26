#include <algorithm>
#include <climits>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <sstream>
#include <stack>
#include <string>
#include <unordered_map>

namespace {

struct TreeNode {
  using Ptr = std::shared_ptr<TreeNode>;
  TreeNode(char val, size_t freq = 0)
      : frequency(freq), value(val), node_count(1) {}
  TreeNode(Ptr l, Ptr r, size_t freq = 0)
      : left(l), right(r), frequency(freq),
        node_count(l->node_count + r->node_count) {}
  bool IsLeaf() const { return left == nullptr && right == nullptr; }
  Ptr left{nullptr};
  Ptr right{nullptr};
  size_t frequency;
  char value;
  size_t node_count;
};

TreeNode::Ptr BuildTree(const std::string &str) {
  std::unordered_map<char, size_t> chars;
  for (auto c : str) {
    auto p = chars.insert(std::make_pair(c, 1));
    if (!p.second)
      p.first->second++;
  }

  std::vector<TreeNode::Ptr> nodes;
  nodes.reserve(chars.size());
  for (auto p : chars)
    nodes.push_back(std::make_shared<TreeNode>(p.first, p.second));

  const auto cmp = [](const auto &lhs, const auto &rhs) {
    return lhs->frequency > rhs->frequency;
  };
  std::priority_queue<TreeNode::Ptr, std::vector<TreeNode::Ptr>, decltype(cmp)>
      huffman_queue{cmp, std::move(nodes)};
  TreeNode::Ptr ret = nullptr;
  while (!huffman_queue.empty()) {
    if (ret) {
      auto next = huffman_queue.top();
      huffman_queue.pop();
      auto freq = ret->frequency + next->frequency;
      huffman_queue.push(std::make_shared<TreeNode>(ret, next, freq));
    }
    ret = huffman_queue.top();
    huffman_queue.pop();
  }
  return ret;
}

using SymbolCodes = std::unordered_map<char, std::string>;

void BuildSymbolCodesImpl(TreeNode::Ptr &tree, SymbolCodes &codes,
                          std::string code) {
  if (tree == nullptr)
    return;
  if (tree->IsLeaf()) {
    codes[tree->value] = std::move(code);
  } else {
    BuildSymbolCodesImpl(tree->left, codes, code + '0');
    BuildSymbolCodesImpl(tree->right, codes, code + '1');
  }
}

SymbolCodes BuildSymbolCodes(TreeNode::Ptr tree) {
  SymbolCodes ret;
  if (tree)
    BuildSymbolCodesImpl(tree, ret, "");
  return ret;
}

class OutBitStream {
  std::streambuf *sbuf_;
  int count_{0};
  unsigned char byte_{0};

public:
  OutBitStream(std::ostream &out) : sbuf_(out.rdbuf()) {}

  ~OutBitStream() {
    if (count_)
      sbuf_->sputc(byte_);
  }

  OutBitStream &operator<<(bool bit) {
    byte_ = (byte_ | bit << count_);
    if (++count_ == CHAR_BIT) {
      sbuf_->sputc(byte_);
      count_ = 0;
      byte_ = 0;
    }
    return *this;
  }

  OutBitStream &operator<<(unsigned char byte) {
    for (int i = 0; i < CHAR_BIT; i++)
      *this << (bool)(byte & (1 << i));
    return *this;
  }
};

class InBitStream {
  std::streambuf *sbuf_;
  int count_{CHAR_BIT};
  unsigned char byte_{0};
  bool is_eof_{false};

public:
  InBitStream(std::istream &in) : sbuf_(in.rdbuf()), count_(CHAR_BIT) {}

  InBitStream &operator>>(bool &bit) {
    if (count_ == CHAR_BIT) {
      if (is_eof_)
        return *this;
      count_ = 0;
      byte_ = sbuf_->sgetc();
      if (sbuf_->snextc() == EOF)
        is_eof_ = true;
    }

    bit = ((byte_ >> count_) & 1);
    count_++;
    return *this;
  }

  InBitStream &operator>>(unsigned char &byte) {
    byte = 0;
    for (int i = 0; i < CHAR_BIT; i++) {
      bool bit;
      *this >> bit;
      byte = (byte | (bit << i));
    }
    return *this;
  }

  operator bool() const { return !(is_eof_ && count_ == CHAR_BIT); }
};

void WriteHeaderImpl(OutBitStream &bitstream, TreeNode::Ptr tree) {
  if (tree == nullptr)
    return;
  if (tree->IsLeaf()) {
    bitstream << true << reinterpret_cast<unsigned char &>(tree->value);
  } else {
    WriteHeaderImpl(bitstream, tree->left);
    WriteHeaderImpl(bitstream, tree->right);
    bitstream << false;
  }
}

std::ostream &WriteHeader(std::ostream &out, TreeNode::Ptr tree) {
  uint16_t node_count = static_cast<uint16_t>(tree->node_count);
  out.write(reinterpret_cast<char *>(&node_count), sizeof(node_count));
  OutBitStream bitstream{out};
  WriteHeaderImpl(bitstream, tree);
  return out;
}

TreeNode::Ptr ReadHeader(std::istream &in) {
  uint16_t node_count;
  if (!in.read(reinterpret_cast<char *>(&node_count), sizeof(node_count)))
    return nullptr;

  InBitStream bitstream{in};
  std::vector<TreeNode::Ptr> worklist;
  bool bit;
  unsigned char byte;
  while (bitstream >> bit) {
    if (bit) {
      bitstream >> byte;
      worklist.push_back(std::make_shared<TreeNode>(byte));
    } else {
      if (worklist.size() < 2)
        return nullptr;
      auto right = worklist.back();
      worklist.pop_back();
      auto left = worklist.back();
      worklist.pop_back();
      worklist.push_back(std::make_shared<TreeNode>(left, right));
      if (worklist.back()->node_count >= node_count)
        break;
    }
  }

  if (worklist.size() == 1)
    return worklist.back();
  return nullptr;
}

bool Encode(std::istream &in, std::ostream &out) {
  std::string str;
  std::copy(std::istreambuf_iterator<char>(in), {}, std::back_inserter(str));
  if (str.empty()) {
    std::cout << "Nothing to compress\n";
    return false;
  }

  auto tree = BuildTree(str);
  if (tree == nullptr)
    throw std::runtime_error("Internal error");

  WriteHeader(out, tree);
  auto codes = BuildSymbolCodes(tree);

  in.clear();
  in.seekg(0, std::ios::beg);

  int bit_count = 0;
  unsigned char byte = 0;
  for (auto buf_it = std::istreambuf_iterator<char>(in);
       buf_it != std::istreambuf_iterator<char>(); ++buf_it) {
    auto code_it = codes.find(*buf_it);
    if (code_it == codes.end())
      throw std::runtime_error("Internal error");

    const auto &code = code_it->second;
    for (auto c : code) {
      byte |= ((c - '0') << bit_count);
      if (++bit_count == CHAR_BIT) {
        out.write(reinterpret_cast<char *>(&byte), sizeof(byte));
        byte = 0;
        bit_count = 0;
      }
    }
  }

  if (bit_count > 0)
    out.write(reinterpret_cast<char *>(&byte), sizeof(byte));

  return true;
}

bool Decode(std::istream &in, std::ostream &out) {
  auto tree = ReadHeader(in);
  if (tree == nullptr) {
    std::cout << "Couldn't read archive header\n";
    return false;
  }

  InBitStream bitstream(in);
  bool bit;
  TreeNode::Ptr cur = tree;

  while (bitstream >> bit) {
    cur = bit ? cur->right : cur->left;

    if (cur == nullptr) {
      std::cout << "Invalid input format\n";
      return false;
    } else if (cur->IsLeaf()) {
      out.write(&cur->value, sizeof(cur->value));
      cur = tree;
    }
  }
  return true;
}

} // namespace

#ifndef PROGRAM_NAME
#define PROGRAM_NAME "compress"
#endif

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cout << "Usage: " << PROGRAM_NAME << " <input file> <output file>\n";
    return 1;
  }

#ifdef BUILD_COMPRESS
  std::fstream input_file(argv[1]);
  std::fstream output_file(argv[2], std::ios::binary | std::fstream::out);
  if (Encode(input_file, output_file))
    return 0;
#else
  std::fstream input_file(argv[1], std::fstream::in | std::fstream::binary);
  std::fstream output_file(argv[2], std::fstream::out);
  if (Decode(input_file, output_file))
    return 0;
#endif

  return 1;
}
