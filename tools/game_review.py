"""Run the shared offline harness for the complete match application."""
import fixed_review as review

review.OUTPUT = review.ROOT / 'output/game-main-20261114'
review.EVIDENCE = review.ROOT / 'docs/evidence/2026-11-14/game-main-20261114'

if __name__ == '__main__':
    review.main()
